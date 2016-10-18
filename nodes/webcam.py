#!/usr/bin/env python
from __future__ import print_function

import os
import threading

import rospy
import std_srvs.srv
from sensor_msgs.msg import CompressedImage, Image

import numpy as np
import cv2

class WebCam():
    """
    A dirt simple broadcasting node of a compressed webcam video stream.
    """
    
    def __init__(self):
        
        rospy.init_node('webcam', log_level=rospy.DEBUG)
        
        self._lock = threading.RLock()
        
        self.image_pub = rospy.Publisher('~image/compressed', CompressedImage, queue_size=1)
        
        self.video_index = int(rospy.get_param("~index", 0))
        
        self.auto_start = int(rospy.get_param("~start", 0))
        
        self.running = False
        
        # Handle of the main processing thread.
        self._camera_thread = None
        
        # Cleanup when termniating the node.
        rospy.on_shutdown(self.shutdown)
        
        rospy.Service('~start', std_srvs.srv.Empty, self.start)
                
        rospy.Service('~stop', std_srvs.srv.Empty, self.stop)
        
        if self.auto_start:
            self.start()
        
        # Start polling the sensors and base controller.
        self.rate = int(rospy.get_param("~rate", 60))
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()
 
    def start(self, msg=None):
        with self._lock:
            if not self.running:
                self.running = True
                self._camera_thread = threading.Thread(target=self._process_video)
                self._camera_thread.daemon = True
                self._camera_thread.start()
 
    def stop(self, msg=None):
        with self._lock:
            if self._camera_thread:
                self.running = False
                self._camera_thread.join()
                self._camera_thread = None
        
    def _process_video(self):
        video_capture = cv2.VideoCapture(self.video_index)
        try:
            while self.running:
                ret, frame = video_capture.read()
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
                self.image_pub.publish(msg)
        except rospy.exceptions.ROSInterruptException:
            self.running = False
        
        if video_capture:
            video_capture.release()

    def shutdown(self):
        rospy.loginfo('Shutting down...')
        self.running = False
        rospy.loginfo('Done.')
        
if __name__ == '__main__':
    WebCam()
