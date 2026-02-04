#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import yaml

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')
        self.bridge = CvBridge()
        
        # Updated topic names
        self.image_pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=1)
        self.fire_pub = rospy.Publisher('/fire_status', String, queue_size=1)
        self.flame_x_pub = rospy.Publisher('/flame_x', Float32, queue_size=1)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Fire detection
        cascade_path = os.path.join(os.path.dirname(__file__), "../resources/fire_detection.xml")
        self.fire_detector = cv2.CascadeClassifier(cascade_path)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Camera read failed")
                continue
                
            # Fire detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fires = self.fire_detector.detectMultiScale(gray, 1.2, 5)
            
            # Publish fire status
            if len(fires) > 0:
                self.fire_pub.publish("fire")
                x_center = fires[0][0] + fires[0][2]/2
                self.flame_x_pub.publish(Float32(x_center))
            else:
                self.fire_pub.publish("no_fire")
            
            # Publish image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = rospy.Time.now()
                self.image_pub.publish(img_msg)
            except Exception as e:
                rospy.logerr(f"Image publish error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        CameraPublisher().run()
    except rospy.ROSInterruptException:
        pass