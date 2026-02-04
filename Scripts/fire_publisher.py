#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os
import asyncio
import websockets
import threading

# WebSocket server class
class WebSocketServer:
    def __init__(self):
        self.current_data = "Initializing fire status..."
        self.loop = asyncio.new_event_loop()
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
    
    def update_data(self, message):
        self.current_data = message
    
    async def handler(self, websocket, path):
        async def send_fire_data():
            while True:
                await websocket.send(self.current_data)
                await asyncio.sleep(0.5)

        async def receive_messages():
            async for message in websocket:
                print(f"Received from client: {message}")
                await websocket.send(f"RPi acknowledged: {message}")

        await asyncio.gather(send_fire_data(), receive_messages())

    async def main_server(self):
        async with websockets.serve(self.handler, "0.0.0.0", 8765):
            await asyncio.Future()

    def run_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.main_server())

# Fire detection node
def fire_detection_node():
    pub = rospy.Publisher('fire_status', String, queue_size=10)
    flame_x_pub = rospy.Publisher('/flame_x', Float32, queue_size=10)
    image_pub = rospy.Publisher('/rgb/image_raw', Image, queue_size=10)

    rospy.init_node('fire_publisher', anonymous=True)
    ws_server = WebSocketServer()
    bridge = CvBridge()

    cascade_path = os.path.join(os.path.dirname(__file__), "../resources/fire_detection.xml")
    fire_detector = cv2.CascadeClassifier(cascade_path)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Camera not accessible")
        return

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        success, img = cap.read()
        if not success:
            rospy.logwarn("Failed to read from camera")
            break

        img = cv2.flip(img, 1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        fire = fire_detector.detectMultiScale(gray, 1.2, 5)
        fire_detected = False
        x_center = None

        for (x, y, w, h) in fire:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 3)
            fire_detected = True
            x_center = x + w / 2
            break

        status = "fire" if fire_detected else "no_fire"
        rospy.loginfo(f"Publishing: {status}")
        pub.publish(status)

        if fire_detected and x_center is not None:
            flame_x_pub.publish(Float32(x_center))
            rospy.loginfo(f"Flame X: {x_center:.2f}")

        # Publish the raw image
        try:
            image_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            image_pub.publish(image_msg)
        except Exception as e:
            rospy.logwarn(f"Failed to convert and publish image: {e}")

        ws_server.update_data(status)

        cv2.imshow('Fire Detection', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        fire_detection_node()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")

