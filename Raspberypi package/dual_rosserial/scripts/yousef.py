#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class CameraPublisher:
    def __init__(self):
        rospy.init_node('fire_camera_publisher', anonymous=True)

        # Publishers
        self.fire_pub = rospy.Publisher('fire_status', String, queue_size=10)
        self.flame_x_pub = rospy.Publisher('/flame_x', Float32, queue_size=10)
        self.image_pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=10)

        self.bridge = CvBridge()

        # Load fire detection cascade
        cascade_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources/fire_detection.xml")
        rospy.loginfo(f"Loading cascade from: {cascade_path}")
        self.fire_detector = cv2.CascadeClassifier(cascade_path)

        if self.fire_detector.empty():
            rospy.logerr(f"Failed to load fire detection cascade at {cascade_path}")
            raise IOError(f"Could not load cascade classifier from {cascade_path}")

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Could not open video device")
            raise IOError("Camera not accessible")

        # Set camera resolution (adjust to your camera's capabilities)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Camera calibration parameters (replace with your actual calibration)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "camera_link"
        self.camera_info.width = 640
        self.camera_info.height = 480
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.K = [525.0, 0.0, 320.0, 
                              0.0, 525.0, 240.0, 
                              0.0, 0.0, 1.0]
        self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.R = [1.0, 0.0, 0.0, 
                              0.0, 1.0, 0.0, 
                              0.0, 0.0, 1.0]
        self.camera_info.P = [525.0, 0.0, 320.0, 0.0, 
                              0.0, 525.0, 240.0, 0.0, 
                              0.0, 0.0, 1.0, 0.0]

    def publish_camera_data(self):
        rate = rospy.Rate(15)  # 15 Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue

            frame = cv2.flip(frame, 1)

            # Fire detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            fires = self.fire_detector.detectMultiScale(gray, 1.2, 5)

            fire_detected = False
            x_center = None

            for (x, y, w, h) in fires:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                fire_detected = True
                x_center = x + w / 2
                break  # Only process the first detection

            # Publish fire status
            status = "fire" if fire_detected else "no_fire"
            self.fire_pub.publish(status)

            if fire_detected and x_center is not None:
                self.flame_x_pub.publish(Float32(x_center))

            # Publish image and camera info
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_link"

                self.camera_info.header.stamp = ros_image.header.stamp

                self.image_pub.publish(ros_image)
                self.camera_info_pub.publish(self.camera_info)
            except Exception as e:
                rospy.logerr(f"Image conversion error: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        publisher = CameraPublisher()
        publisher.publish_camera_data()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {e}")
    finally:
        if 'publisher' in locals() and hasattr(publisher, 'cap'):
            publisher.cap.release()
