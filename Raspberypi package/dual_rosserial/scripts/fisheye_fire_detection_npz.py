#!/usr/bin/env python3 
import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class FisheyeFireDetection:
    def __init__(self):
        rospy.init_node('fisheye_fire_detector')
        self.bridge = CvBridge()
        
        # Load fisheye calibration from .npz file
        calib_path = os.path.join(os.path.dirname(__file__), 'fisheye_calib.npz')
        self.calib_data = np.load(calib_path)
        self.K = self.calib_data['K']
        self.D = self.calib_data['D']

        # Publishers
        self.fire_pub = rospy.Publisher('fire_status', String, queue_size=10)
        self.flame_x_pub = rospy.Publisher('/flame_x', Float32, queue_size=10)
        self.image_pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=10)
        self.cam_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)

        # Setup CameraInfo message
        self.cam_info = CameraInfo()
        self.cam_info.width = 640  # Update with your resolution
        self.cam_info.height = 480
        self.cam_info.K = self.K.flatten().tolist()
        self.cam_info.D = self.D.flatten().tolist()
        self.cam_info.distortion_model = "fisheye"

        # Fire detection setup
        cascade_path = os.path.join(os.path.dirname(__file__), "../resources/fire_detection.xml")
        self.fire_cascade = cv2.CascadeClassifier(cascade_path)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Could not open video device")
            exit(1)

    def undistort_frame(self, frame):
        """Fisheye undistortion using loaded calibration"""
        h, w = frame.shape[:2]
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, (w,h), np.eye(3), balance=0.0)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), new_K, (w,h), cv2.CV_16SC2)
        return cv2.remap(frame, map1, map2, 
                        interpolation=cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue
            
            # Process frame
            frame = self.undistort_frame(frame)
            frame = cv2.flip(frame, 1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Fire detection
            fires = self.fire_cascade.detectMultiScale(gray, 1.2, 5)
            
            # Publish results
            if len(fires) > 0:
                x, y, w, h = fires[0]
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,0,255), 2)
                self.fire_pub.publish("fire")
                self.flame_x_pub.publish(Float32(x + w/2))
            else:
                self.fire_pub.publish("no_fire")
            
            # Publish ROS messages
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = "camera_link"
                self.image_pub.publish(img_msg)
                
                self.cam_info.header = img_msg.header
                self.cam_info_pub.publish(self.cam_info)
            except Exception as e:
                rospy.logerr(f"Publish error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = FisheyeFireDetection()
        node.run()
    except rospy.ROSInterruptException:
        pass