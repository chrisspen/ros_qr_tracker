#!/usr/bin/env python
from __future__ import print_function

import os
import time
import threading
import io
import traceback
from math import sqrt, acos, pi

import numpy as np
from PIL import Image as PilImage

import rospy
import std_srvs.srv
from std_srvs.srv import EmptyResponse
from sensor_msgs.msg import CompressedImage, Image

import libzbar as zb
import cv2
from cv_bridge import CvBridge, CvBridgeError

from ros_qr_tracker.msg import Percept
from ros_qr_tracker.srv import AddTarget, AddTargetResponse, SetTarget, SetTargetResponse

def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class Modes:
    PUSH = 1 # faster (~30 fps) but needs more CPU (default)
    PULL = 2 # slower (~10fps) but uses less CPU

class QRTracker():
    
    def __init__(self):
        
        rospy.init_node('qr_tracker', log_level=rospy.DEBUG)
        
        self._lock = threading.RLock()
        
        self.camera_topic = rospy.get_param("~topic", '/camera/compressed')
        
        self.qr_pub = rospy.Publisher('~matches', Percept, queue_size=10)
        
        self.images_pub = rospy.Publisher('~images/compressed', CompressedImage, queue_size=1)
        
        self.show_matches = int(rospy.get_param("~show_matches", 1))
        
        self.auto_start = int(rospy.get_param("~start", 0))
        
        self.processing_mode = int(rospy.get_param('~mode', Modes.PULL))
        
        # This is used to estimate distance from QR codes.
        self.focal_length = float(rospy.get_param('~focal_length', 640))

        # Text that we search for in a QR code.
        # If this is set, and a QR code is detected that does not contain this text,
        # it will be ignored.
        self._targets = [_ for _ in rospy.get_param("~targets", '').split(',') if _.strip()]
        
        self.running = False
        
        self._frames = 0
        
        self.last_msg = None
        
        self._t0 = None
        
        self.bridge = CvBridge()
        
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
            
            if self.processing_mode == Modes.PULL:
            
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
            rospy.loginfo('Set target "%s".', msg.data)
        return SetTargetResponse()
 
    def add_target(self, msg):
        with self._lock:
            self._targets.append(msg.data)
            rospy.loginfo('Appended target "%s".', msg.data)
            rospy.loginfo('There are now %i targets.', len(self._targets))
        return AddTargetResponse()
 
    def clear_target(self, msg=None):
        with self._lock:
            self._targets = []
            rospy.loginfo('Targets cleared.')
        return EmptyResponse()

    def get_image(self):
        try:
            if 'compressed' in self.camera_topic:
                msg = rospy.wait_for_message(self.camera_topic, CompressedImage, timeout=1)
            else:
                msg = rospy.wait_for_message(self.camera_topic, Image, timeout=1)
            with self._lock:
                self.last_msg = msg
            return msg
        except rospy.exceptions.ROSException:
            return
            
    def normalize_compressed_image(self, msg):
        return PilImage.open(io.BytesIO(bytearray(msg.data)))
        
    def normalize_image_cv2(self, msg):
        if isinstance(msg, CompressedImage):
            pil_image = self.normalize_compressed_image(msg)
            cv_image = np.array(pil_image)
            return cv_image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
        return cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
         
    def normalize_image_pil(self, msg):
        if isinstance(msg, CompressedImage):
            return self.normalize_compressed_image(msg)
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
            
            if msg:
                with self._lock:
                    self.last_msg = msg
            
            while self.running:
                pil_img = self.get_image_pil(msg=msg)
                if pil_img:
                    
                    if self.show_matches:
                        try:
                            cv_img = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
                        except Exception as exc:
                            rospy.log_error('Unable to read image: %s', exc)
                            traceback.print_exc()
                    
                    # Track FPS.
                    if not self._frames:
                        self._t0 = time.time()
                    self._frames += 1
                    if not self._frames % 50:
                        rospy.loginfo('fps: %s', self.fps)
                        
                    width, height = pil_img.size
                    centerpoint = width/2, height/2
                    cp_size = 10
                    # print('size:', width, height)
                    matches = zb.Image.from_im(pil_img).scan()
                    
                    if matches:
                        for match in matches:
                            
                            with self._lock:
                                if self._targets and match.data not in self._targets:
                                    continue
                            
                            tl, bl, br, tr = match.locator
                            match_cp = ((tl[0]+bl[0]+br[0]+tr[0])/4, (tl[1]+bl[1]+br[1]+tr[1])/4)
                            
                            centerpoint_dist = dist((match_cp[0], 0), (centerpoint[0], 0))
                            outline_color = (0, 0, 255) # bgr
                            if centerpoint_dist < 10:
                                outline_color = (0, 255, 0) # bgr
                            
                            distance_mm = '?'
                            distance_meters = None
                            camera_angle_radians = None
                            known_width_mm = None
                            known_height_mm = None
                            # print('data:',match.data)
                            if '=' in match.data:
                                data_pairs = dict(part.split('=')[:2] for part in match.data.split(','))
                                if 'w' in data_pairs:
                                    known_width_mm = int(data_pairs['w'])
                                if 'h' in data_pairs:
                                    known_height_mm = int(data_pairs['h'])

                            if self.show_matches:
                                
                                # Draw outline.
                                cv2.line(cv_img, tl, bl, outline_color, 3)
                                cv2.line(cv_img, bl, br, outline_color, 3)
                                cv2.line(cv_img, br, tr, outline_color, 3)
                                cv2.line(cv_img, tr, tl, outline_color, 3)

                                # Draw outline center.
                                cv2.line(cv_img, (match_cp[0]-cp_size, match_cp[1]-cp_size), (match_cp[0]+cp_size, match_cp[1]+cp_size), (0, 0, 255), 2)
                                cv2.line(cv_img, (match_cp[0]-cp_size, match_cp[1]+cp_size), (match_cp[0]+cp_size, match_cp[1]-cp_size), (0, 0, 255), 2)

                                # Draw image center.
                                cv2.line(
                                    cv_img,
                                    (centerpoint[0]-cp_size, centerpoint[1]-cp_size), (centerpoint[0]+cp_size, centerpoint[1]+cp_size), (255, 0, 0), 2)
                                cv2.line(
                                    cv_img,
                                    (centerpoint[0]-cp_size, centerpoint[1]+cp_size), (centerpoint[0]+cp_size, centerpoint[1]-cp_size), (255, 0, 0), 2)

                                # Calculate direction of viewing angle.
                                true_bl, true_br = sorted([tl, bl, br, tr], key=lambda p: p[1])[:2]
                                bottom_hyp = sqrt((true_bl[0] - true_br[0])**2 + (true_bl[1] - true_br[1])**2)
                                bottom_x = true_bl[0] - true_br[0]
                                camera_angle0 = acos(bottom_x/float(bottom_hyp))
                                angle_direction = 1
                                if camera_angle0 > pi/2:
                                    angle_direction = -1
                                distances = [dist(tl, tr), dist(tr, br), dist(br, bl), dist(bl, tl)]
                                
                                # Calculate camera viewing angle.
                                # When camera is straight on, all side lengths should be equal.
                                # When camera is at a perfect 90 degree angle to the code (it won't be visible), by hypothetically, two lengths should be zero,
                                # so the angle of view should be proportional to the ratio between the largest and smallest side lengths.
                                max_distance = max(distances)
                                min_distance = min(distances)
                                camera_angle_degrees = (90 - min_distance/float(max_distance)*90) * angle_direction
                                camera_angle_radians = camera_angle_degrees * pi / 180.
                                
                                if known_width_mm:
                                    pixel_width1 = dist(tl, tr)
                                    pixel_width2 = dist(bl, br)
                                    pixel_width = (pixel_width1 + pixel_width2)/2.
                                    distance_mm = int((known_width_mm * self.focal_length) / float(pixel_width))
                                    distance_meters = distance_mm * 1/1000.

                                    min_point = min(tl, bl, br, tr, key=lambda p: p[0]**2 + p[1]**2)

                                font = cv2.FONT_HERSHEY_SIMPLEX
                                bottomLeftCornerOfText_distance = (min_point[0], -22/2 + min_point[1])
                                # bottomLeftCornerOfText_angle = (min_point[0], -22/2 + min_point[1] + 30)
                                # print('bottomLeftCornerOfText:', bottomLeftCornerOfText)
                                fontScale = 1 # scale=1 => font_height=22px, note text is by default vertically centered
                                fontColor = (0, 0, 255)
                                white = (255, 255, 255)
                                lineType = 2

                                for point_label, point_coords in zip(('tl', 'tr', 'bl', 'br'), (tl, tr, bl, br)):
                                    cv2.putText(cv_img, point_label, 
                                        point_coords, 
                                        font, 
                                        fontScale,
                                        (255, 0, 0),
                                        lineType)

                                cv2.putText(cv_img, 'Distance (mm) = %i' % distance_mm,
                                    (min_point[0], -22/2 + min_point[1] + 30*1), 
                                    font, 
                                    fontScale*0.5,
                                    white,
                                    lineType)

                                cv2.putText(cv_img, 'Angle (deg) = %.02f' % camera_angle_degrees,
                                    (min_point[0], -22/2 + min_point[1] + 30*2), 
                                    font, 
                                    fontScale*0.5,
                                    white,
                                    lineType)

                                cv2.putText(cv_img, 'X = %i' % match_cp[0],
                                    (min_point[0], -22/2 + min_point[1] + 30*3),
                                    font, 
                                    fontScale*0.5,
                                    white,
                                    lineType)
                                    
                                cv2.putText(cv_img, 'Y = %i' % match_cp[1],
                                    (min_point[0], -22/2 + min_point[1] + 30*4),
                                    font, 
                                    fontScale*0.5,
                                    white,
                                    lineType)

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
                            percept.distance = distance_meters
                            percept.deflection_angle = camera_angle_radians
                            percept.data = match.data
                            self.qr_pub.publish(percept)
                    
                    if self.show_matches:
                        self.images_pub.publish(self.bridge.cv2_to_compressed_imgmsg(cv_img, dst_format='jpg'))
                
                # If we're not a thread and have been given a single message to process, exit immediately.
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
