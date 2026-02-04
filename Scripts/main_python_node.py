#!/usr/bin/env python3   
import rospy
from std_msgs.msg import Float32MultiArray, String, Int32MultiArray, Float32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image  # Import Image message type
import cv2
from cv_bridge import CvBridge
import asyncio
import websockets
import threading
import time
import smtplib
from email.message import EmailMessage

class WebSocketServer:
    def __init__(self):
        self.current_data = "Initializing sensor data..."
        self.pump_status = "OFF"
        self.flame_sensor_data = "(0, 0, 0, 0, 0)"
        self.sensor_readings = "(Front: -, Left: -, Right: -)"  # Initialize sensor readings
        self.last_sent_data = None  # Track last sent data to detect changes
        self.lock = threading.Lock()  # To ensure thread-safe updates
        self.loop = asyncio.new_event_loop()
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        self.image_data = None  # Store the latest image data
        self.bridge = CvBridge()  # For converting ROS Image messages to OpenCV format

    def update_data(self, message):
        with self.lock:
            self.current_data = message

    def update_pump_status(self, status):
        with self.lock:
            self.pump_status = "ON" if status else "OFF"

    def update_flame_sensor(self, data):
        with self.lock:
            self.flame_sensor_data = str(data)

    def update_sensor_readings(self, front, left, right):
        with self.lock:
            self.sensor_readings = f"(Front: {front:.1f}, Left: {left:.1f}, Right: {right:.1f})"

    def update_image_data(self, image_msg):
        with self.lock:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
                # Encode the image as a JPEG to send over WebSocket
                _, jpeg_image = cv2.imencode('.jpg', cv_image)
                self.image_data = jpeg_image.tobytes()
            except Exception as e:
                rospy.logerr(f"Failed to process image: {e}")
                self.image_data = None

    async def handler(self, websocket, path):
        async def send_updates():
            while True:
                with self.lock:
                    combined_data = (
                        f"{self.current_data}\n"
                        f"Pump Status: {self.pump_status}\n"
                        f"Flame Sensor Readings: {self.flame_sensor_data}\n"
                        f"Sensor Readings: {self.sensor_readings}"
                    )
                    if combined_data != self.last_sent_data:  # Send only if data has changed
                        await websocket.send(combined_data)
                        self.last_sent_data = combined_data
                    if self.image_data:  # Send image data if available
                        await websocket.send(self.image_data)
                await asyncio.sleep(0.2)  # Adjusted delay for efficient updates

        async def receive_messages():
            async for message in websocket:
                print(f"Received from client: {message}")
                await websocket.send(f"RPi acknowledged: {message}")

        await asyncio.gather(send_updates(), receive_messages())

    async def main_server(self):
        async with websockets.serve(self.handler, "0.0.0.0", 8765):
            await asyncio.Future()

    def run_server(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.main_server())

# ---------------- Global State ----------------
last_move_forward_time = None
scanning = False
latest_sensor_data = None
fire_stop = False
camera_fire_detected = False
orienting_to_flame = False
latest_flame_readings = None
global_ws_server = None
pub_decision = None
cmd_vel_pub = None
pump_control_pub = None
encoder_stuck = False

# ---------------- Vision Control Parameters ----------------
Kp_ang = 0.002
v_forward = 2.0
image_width = 640
u0 = image_width / 2.0

# ---------------- Motion Helper ----------------
def send_cmd_vel(linear, angular):
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    cmd_vel_pub.publish(twist)

# ---------------- Pump Control Callback ----------------
def pump_control_callback(msg):
    global global_ws_server
    pump_status = "ON" if msg.data else "OFF"
    rospy.loginfo(f"Pump Control: {pump_status}")
    if global_ws_server:
        global_ws_server.update_pump_status(msg.data)
        global_ws_server.update_data(f"Pump Control: {pump_status}")

# ---------------- Modified Logging ----------------
def log_and_update_ws(message):
    global global_ws_server
    rospy.loginfo(message)
    if global_ws_server:
        global_ws_server.update_data(message)

# ---------------- Flame Pixel Callback ----------------
def flame_pixel_callback(msg):
    global camera_fire_detected, fire_stop
    if not camera_fire_detected or fire_stop:
        return

    u = msg.data
    e = u0 - u
    omega = Kp_ang * e

    twist = Twist()
    twist.linear.x = v_forward
    twist.angular.z = omega
    cmd_vel_pub.publish(twist)
    log_and_update_ws(f"Vision centering: pixel error={e:.1f}, omega={omega:.3f}")

# ---------------- Fire Handling ----------------
def reset_fire_handling(pub_decision):
    global fire_stop, camera_fire_detected, orienting_to_flame, last_move_forward_time

    log_and_update_ws("Resetting fire state. Resuming search and avoidance.")
    fire_stop = False
    camera_fire_detected = False
    orienting_to_flame = False
    last_move_forward_time = None

    threading.Thread(target=scan_pattern, args=(pub_decision, global_ws_server)).start()

def scan_pattern(pub_decision, ws_server): 
    global scanning, latest_sensor_data, fire_stop, encoder_stuck
    if fire_stop:
        return

    scanning = True
    log_and_update_ws("=== SCANNING STARTED ===")

    try:
        while True:
            if fire_stop:
                return

            # Check if the robot is stuck
            if encoder_stuck:
                log_and_update_ws("Robot is stuck! Moving backward and turning.")

                # Move backward for 2 seconds
                back_start = time.time()
                while time.time() - back_start < 1:
                    send_cmd_vel(-2.1, 0.0)
                    rospy.sleep(0.1)  # Small sleep to avoid spamming

                # Turn for 2 seconds
                turn_start = time.time()
                while time.time() - turn_start < 1.0:
                    send_cmd_vel(0.0, 10.1)
                    rospy.sleep(0.1)

                # Stop the robot
                send_cmd_vel(0.0, 0.0)
                encoder_stuck = False

            move_start = time.time()
            while time.time() - move_start < 3.0:
                if fire_stop:
                    return

                if encoder_stuck:
                    log_and_update_ws("Robot is stuck! Moving backward and turning.")

                    # Move backward for 2 seconds
                    back_start = time.time()
                    while time.time() - back_start < 1.0:
                        send_cmd_vel(-2.1, 0.0)
                        rospy.sleep(0.1)  # Small sleep to avoid spamming

                    # Turn for 2 seconds
                    turn_start = time.time()
                    while time.time() - turn_start < 1.0:
                        send_cmd_vel(0.0, 10.1)
                        rospy.sleep(0.1)

                    # Stop the robot
                    send_cmd_vel(0.0, 0.0)
                    encoder_stuck = False

                if latest_sensor_data and latest_sensor_data[0] < 45 and latest_sensor_data[0] != -1:
                    log_and_update_ws("SCAN INTERRUPTED BY OBSTACLE")
                    scanning = False
                    return
                send_cmd_vel(2.0, 0.0)
                time.sleep(0.2)

            turn_start = time.time()
            while time.time() - turn_start < 1.0:
                if fire_stop:
                    return

                if encoder_stuck:
                    log_and_update_ws("Robot is stuck! Moving backward and turning.")

                    # Move backward for 2 seconds
                    back_start = time.time()
                    while time.time() - back_start < 1.0:
                        send_cmd_vel(-2.1, 0.0)
                        rospy.sleep(0.1)  # Small sleep to avoid spamming

                    # Turn for 2 seconds
                    turn_start = time.time()
                    while time.time() - turn_start < 1.0:
                        send_cmd_vel(0.0, 10.1)
                        rospy.sleep(0.1)

                    # Stop the robot
                    send_cmd_vel(0.0, 0.0)
                    encoder_stuck = False

                if latest_sensor_data and latest_sensor_data[0] < 45 and latest_sensor_data[0] != -1:
                    log_and_update_ws("SCAN INTERRUPTED BY OBSTACLE")
                    scanning = False
                    return
                send_cmd_vel(0.0, 10.1)
                time.sleep(0.05)

            move_start = time.time()
            while time.time() - move_start < 3.0:
                if fire_stop:
                    return

                if encoder_stuck:
                    log_and_update_ws("Robot is stuck! Moving backward and turning.")

                    # Move backward for 2 seconds
                    back_start = time.time()
                    while time.time() - back_start < 1.0:
                        send_cmd_vel(-3.1, 0.0)
                        rospy.sleep(0.1)  # Small sleep to avoid spamming

                    # Turn for 2 seconds
                    turn_start = time.time()
                    while time.time() - turn_start < 1.0:
                        send_cmd_vel(0.0, 10.1)
                        rospy.sleep(0.1)

                    # Stop the robot
                    send_cmd_vel(0.0, 0.0)
                    encoder_stuck = False

                if latest_sensor_data and latest_sensor_data[0] < 45 and latest_sensor_data[0] != -1:
                    log_and_update_ws("SCAN INTERRUPTED BY OBSTACLE")
                    scanning = False
                    return
                send_cmd_vel(2.0, 0.0)
                time.sleep(0.2)

            turn_start = time.time()
            while time.time() - turn_start < 1.0:
                if fire_stop:
                    return

                if encoder_stuck:
                    log_and_update_ws("Robot is stuck! Moving backward and turning.")

                    # Move backward for 2 seconds
                    back_start = time.time()
                    while time.time() - back_start < 1.0:
                        send_cmd_vel(-3.1, 0.0)
                        rospy.sleep(0.1)  # Small sleep to avoid spamming

                    # Turn for 2 seconds
                    turn_start = time.time()
                    while time.time() - turn_start < 1.0:
                        send_cmd_vel(0.0, 10.1)
                        rospy.sleep(0.1)

                    # Stop the robot
                    send_cmd_vel(0.0, 0.0)
                    encoder_stuck = False

                if latest_sensor_data and latest_sensor_data[0] < 45 and latest_sensor_data[0] != -1:
                    log_and_update_ws("SCAN INTERRUPTED BY OBSTACLE")
                    scanning = False
                    return
                send_cmd_vel(0.0, -10.1)
                time.sleep(0.1)

    finally:
        scanning = False
        log_and_update_ws("=== SCANNING FINISHED OR INTERRUPTED ===")

def ros_callback(data, args):
    global last_move_forward_time, scanning, latest_sensor_data, fire_stop, camera_fire_detected, global_ws_server

    if fire_stop:
        return

    pub_decision, ws_server = args
    front, left, right = data.data[0], data.data[1], data.data[2]
    latest_sensor_data = (front, left, right)

    # Update WebSocket server with the latest sensor readings
    if ws_server:
        ws_server.update_sensor_readings(front, left, right)

    sensor_msg = (
        "=== Sensor Readings ===\n"
        f"Front: {front:.1f} cm\n"
        f"Left:  {left:.1f} cm\n"
        f"Right: {right:.1f} cm\n"
    )

    if ws_server:
        ws_server.update_data(sensor_msg)

    if camera_fire_detected:
        action_msg = "CAMERA: Fire detected — centering with vision."
        full_message = sensor_msg + action_msg
        log_and_update_ws(full_message)
        return

    action_msg = ""

    if front == -1 or front >= 45:
        action_msg = "ACTION: Moving forward (clear path)"
        send_cmd_vel(2.0, 0.0)

        if not scanning:
            now = time.time()
            if last_move_forward_time is None:
                last_move_forward_time = now
            elif now - last_move_forward_time >= 2.0:
                threading.Thread(target=scan_pattern, args=(pub_decision, ws_server)).start()
                last_move_forward_time = now
        else:
            last_move_forward_time = time.time()
    else:
        action_msg = "ACTION: Obstacle detected — evaluating..."
        if left == -1 and right == -1:
            action_msg += "\nDECISION: Turning right (no side data)"
            send_cmd_vel(0.0, -10.1)
        elif left > right:
            action_msg += "\nDECISION: Turning right (better path)"
            send_cmd_vel(0.0, -10.1)
        else:
            action_msg += "\nDECISION: Turning left (better path)"
            send_cmd_vel(0.0, 10.1)
        last_move_forward_time = None

    full_message = sensor_msg + action_msg
    log_and_update_ws(full_message)

def send_email_notification(subject, content):
    email_address = "mahmoudaldwakhly53@gmail.com"
    app_password = "vjsy essk rzkw uhta"

    msg = EmailMessage()
    msg["Subject"] = subject
    msg["From"] = email_address
    msg["To"] = email_address
    msg.set_content(content)

    try:
        with smtplib.SMTP_SSL("smtp.gmail.com", 465) as smtp:
            smtp.login(email_address, app_password)
            smtp.send_message(msg)
        rospy.loginfo("Email notification sent successfully.")
    except Exception as e:
        rospy.logerr(f"Failed to send email notification: {e}")

def handle_flame_centered(): 
    global orienting_to_flame, latest_flame_readings
    log_and_update_ws("Oriented to flame!")
    send_cmd_vel(0.0, 0.0)
    log_and_update_ws("Stopped to fire")
    log_and_update_ws("Starting water")
    send_email_notification("Fire Detected", "The robot has detected a fire and is attempting to extinguish it.")
    rospy.sleep(2.0)
    pub_decision.publish("pump")
    pump_control_pub.publish(True)

    log_and_update_ws("Waiting for flame to be fully extinguished...")
    while latest_flame_readings and not all(v == 0 for v in latest_flame_readings):
        rospy.sleep(0.5)

    send_cmd_vel(0.0, 0.0)
    pump_control_pub.publish(False)
    log_and_update_ws("No flame detected anymore — resuming search.")
    send_email_notification("Fire Extinguished", "The robot has successfully extinguished the fire.")
    rospy.sleep(3.0)
    log_and_update_ws("Turning around to resume search.")
    send_cmd_vel(0.0, 10.1)
    rospy.sleep(3.0)
    send_cmd_vel(0.0, 0.0)
    rospy.sleep(1.0)
    orienting_to_flame = False
    reset_fire_handling(pub_decision)

def flame_callback(data):
    global fire_stop, latest_flame_readings, orienting_to_flame, pub_decision, global_ws_server
    flame_readings = tuple(data.data)
    latest_flame_readings = flame_readings
    if global_ws_server:
        global_ws_server.update_flame_sensor(flame_readings)
    log_and_update_ws(f"Flame Sensor Readings: {flame_readings}")

    if any(val == 1 for val in flame_readings) and not fire_stop:
        log_and_update_ws("🔥 Fire detected by sensors! Stopping.")
        fire_stop = True
        send_cmd_vel(0.0, 0.0)

        time.sleep(2)
        threading.Thread(target=orient_to_flame, args=(pub_decision,)).start()

def orient_to_flame(pub_decision): 
    global orienting_to_flame, latest_flame_readings
    orienting_to_flame = True
    log_and_update_ws("=== ORIENTING TO FLAME ===")

    while orienting_to_flame:
        if latest_flame_readings[1:4] == (1, 1, 1):
            handle_flame_centered()
            break
        elif latest_flame_readings == (0, 0, 1, 0, 0):
            log_and_update_ws("Center sensor active — Approaching target.")
            send_cmd_vel(1.0, 0.0)
            time.sleep(0.2)
            send_cmd_vel(0.0, 0.0)
        elif latest_flame_readings:
            left_sum = latest_flame_readings[0] + latest_flame_readings[1]
            right_sum = latest_flame_readings[3] + latest_flame_readings[4]
            if left_sum > right_sum:
                send_cmd_vel(0.0, -8.1)   # left 
                time.sleep(0.4)
                send_cmd_vel(0.0, 0.0)    # Right 
            else:
                send_cmd_vel(0.0, 8.1)
                time.sleep(0.4)
                send_cmd_vel(0.0, 0.0)
        time.sleep(0.2)

def camera_fire_callback(msg):
    global camera_fire_detected
    camera_fire_detected = (msg.data == "fire")
    if camera_fire_detected:
        log_and_update_ws("🔥 Fire detected by camera — enabling vision tracking.")

# ---------------- Encoder Stuck Callback ----------------
def encoder_stuck_callback(msg):
    global encoder_stuck
    encoder_stuck = msg.data
    log_and_update_ws(f"Encoder Stuck: {encoder_stuck}")

def image_callback(msg):
    global global_ws_server
    if global_ws_server:
        global_ws_server.update_image_data(msg)

# ---------------- Main ----------------
def main():
    global global_ws_server, pub_decision, cmd_vel_pub, pump_control_pub
    ws_server = WebSocketServer()
    global_ws_server = ws_server

    rospy.init_node('obstacle_avoidance', anonymous=True)
    pub_decision = rospy.Publisher("esp2", String, queue_size=10)
    cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    pump_control_pub = rospy.Publisher("/pump_control", Bool, queue_size=10)

    rospy.Subscriber("esp1", Float32MultiArray, ros_callback, (pub_decision, ws_server))
    rospy.Subscriber("flame_sensor", Int32MultiArray, flame_callback)
    rospy.Subscriber("fire_status", String, camera_fire_callback)
    rospy.Subscriber("/flame_x", Float32, flame_pixel_callback)
    rospy.Subscriber("/encoder_stuck", Bool, encoder_stuck_callback)
    rospy.Subscriber("/pump_control", Bool, pump_control_callback)  # Subscribe to pump control topic
    rospy.Subscriber("/rgb/image_raw", Image, image_callback)  # Subscribe to RGB image topic

    log_and_update_ws("🚗 Obstacle avoidance + 🔥 fire detection + 📷 vision tracking + 💧 pump control ready.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
