#!/usr/bin/env python
from __future__ import print_function

import os
import time
import threading
import io

import rospy
import std_srvs.srv
from std_srvs.srv import EmptyResponse
from sensor_msgs.msg import CompressedImage, Image

import numpy
import libzbar as zb
import cv2
from PIL import Image as PilImage

from ros_qr_tracker.msg import Percept
from ros_qr_tracker.srv import AddTarget, AddTargetResponse, SetTarget, SetTargetResponse

class Modes:
    PUSH = 1 # faster (~30 fps) but needs more CPU (default)
    POLL = 2 # slower (~10fps) but uses less CPU

class QRTracker():
    
    def __init__(self):
        
        rospy.init_node('qr_tracker', log_level=rospy.DEBUG)
        
        self._lock = threading.RLock()
        
        self.camera_topic = rospy.get_param("~topic", '/camera/compressed')
        
        self.qr_pub = rospy.Publisher('~matches', Percept, queue_size=10)
        
        self.auto_start = int(rospy.get_param("~start", 0))
        
        self.processing_mode = int(rospy.get_param('~mode', Modes.PUSH))
        
        self.running = False
        
        self._frames = 0
        
        self._t0 = None
        
        # Text that we search for in a QR code.
        # If this is set, and a QR code is detected that does not contain this text,
        # it will be ignored.
        self._targets = [_ for _ in rospy.get_param("~targets", '').split(',') if _.strip()]
        
        # Handle of the main processing thread.
        self._camera_thread = None
        
        # Handle of the image subscriber.
        self._camera_sub = None
        
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        rospy.Service('~start', std_srvs.srv.Empty, self.start)

        rospy.Service('~stop', std_srvs.srv.Empty, self.stop)

        rospy.Service('~set_target', SetTarget, self.set_target)

        rospy.Service('~add_target', AddTarget, self.add_target)

        rospy.Service('~clear_target', std_srvs.srv.Empty, self.clear_target)
        
        if self.auto_start:
            self.start()
        
        # Start polling the sensors and base controller
        self.rate = int(rospy.get_param("~rate", 60))
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # Publish all sensor values on a single topic for convenience
#             now = rospy.Time.now()
            r.sleep()
 
    @property
    def fps(self):
        return self._frames/float(time.time() - self._t0)
 
    def start(self, msg=None):
        with self._lock:
            self.running = True
            
            if self.processing_mode == Modes.POLL:
            
                # Launch thread to poll.
                if not self._camera_thread:
                    self._camera_thread = threading.Thread(target=self._process_video)
                    self._camera_thread.daemon = True
                    self._camera_thread.start()

            elif self.processing_mode == Modes.PUSH:
    
                # Launch thread to push.
                if not self._camera_sub:
                    if 'compressed' in self.camera_topic:
                        self._camera_sub = rospy.Subscriber(
                            self.camera_topic, CompressedImage, self._process_video)
                    else:
                        self._camera_sub = rospy.Subscriber(
                            self.camera_topic, Image, self._process_video)
                            
            else:
                raise NotImplementedError
                
            rospy.loginfo('Processing started.')
        return EmptyResponse()
 
    def stop(self, msg=None):
        with self._lock:
            
            if self._camera_thread:
                self.running = False
                self._camera_thread.join()
                self._camera_thread = None
            
            if self._camera_sub:
                if 'compressed' in self.camera_topic:
                    self._camera_sub = rospy.Subscriber(
                        self.camera_topic, CompressedImage, self._process_video)
                else:
                    self._camera_sub = rospy.Subscriber(
                        self.camera_topic, Image, self._process_video)
                self._camera_sub.unregister()
                self._camera_sub = None
            
            rospy.loginfo('Processing stopped.')
            self._frames = 0
            self._t0 = None
        return EmptyResponse()
 
    def set_target(self, msg):
        with self._lock:
            self._targets = [msg.data]
            rospy.loginfo('Set target "%s".' % msg.data)
        return SetTargetResponse()
 
    def add_target(self, msg):
        with self._lock:
            self._targets.append(msg.data)
            rospy.loginfo('Appended target "%s".' % msg.data)
            rospy.loginfo('There are now %i targets.' % len(self._targets))
        return AddTargetResponse()
 
    def clear_target(self, msg=None):
        with self._lock:
            self._targets = []
            rospy.loginfo('Targets cleared.')
        return EmptyResponse()
    
    def get_image(self):
        try:
            if 'compressed' in self.camera_topic:
                return rospy.wait_for_message(self.camera_topic, CompressedImage, timeout=1)
            else:
                return rospy.wait_for_message(self.camera_topic, Image, timeout=1)
        except rospy.exceptions.ROSException:
            return
            
    def normalize_compressed_image(self, msg):
        return PilImage.open(io.BytesIO(bytearray(msg.data)))
        
    def normalize_image_cv2(self, msg):
        if isinstance(msg, CompressedImage):
            pil_image = self.normalize_compressed_image(msg)
            cv_image = numpy.array(pil_image)
            return cv_image
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
            return cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
         
    def normalize_image_pil(self, msg):
        if isinstance(msg, CompressedImage):
            return self.normalize_compressed_image(msg)
        else:
            cv_image = self.normalize_image_cv2(msg)
            return PilImage.fromarray(cv_image)
    
    def get_image_pil(self, msg=None):
        msg = msg or self.get_image()
        if msg:
            return self.normalize_image_pil(msg)
        
    def get_image_cv2(self, msg=None):
        msg = msg or self.get_image()
        if msg:
            return self.normalize_image_cv2(msg)
        
    def _process_video(self, msg=None):
        try:
            while self.running:
                pil_img = self.get_image_pil(msg=msg)
                if pil_img:
                    
                    # Track FPS.
                    if not self._frames:
                        self._t0 = time.time()
                    self._frames += 1
                    if not self._frames % 50:
                        rospy.loginfo('fps: {}'.format(self.fps))
                        
                    width, height = pil_img.size
                    matches = zb.Image.from_im(pil_img).scan()
                    if matches:
                        for match in matches:
                            
                            with self._lock:
                                if self._targets and match.data not in self._targets:
                                    continue
                            
                            tl, bl, br, tr = match.locator
                            
                            percept = Percept()
                            percept.frame = self.camera_topic
                            percept.type = match.type
                            percept.quality = match.quality
                            percept.a = list(tl)
                            percept.b = list(bl)
                            percept.c = list(br)
                            percept.d = list(tr)
                            percept.width = width
                            percept.height = height
                            percept.data = match.data
                            
                            self.qr_pub.publish(percept)
                
                # If we're not a thread and have been given a single message to process,
                # exit immediately.
                if msg:
                    break
                            
        except rospy.exceptions.ROSInterruptException:
            pass
        finally:
            if not msg:
                self.running = False

    def shutdown(self):
        rospy.loginfo('Shutting down...')
        self.running = False
        rospy.loginfo('Done.')
        
if __name__ == '__main__':
    QRTracker()
