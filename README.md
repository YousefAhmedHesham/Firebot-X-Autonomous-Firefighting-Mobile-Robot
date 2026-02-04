
https://github.com/user-attachments/assets/6fc2ff6c-5517-43b5-bdf2-2067a06871b2
# Firebot X: Fully Autonomous Firefighting Robot 🚒🤖


## 📖 Project Overview
**Firebot X** is a **fully autonomous** mobile robot designed to detect, track, and extinguish fires in a simulated environment.
this system utilizes a search-and-extinguish logic to freely navigate its environment without pre-defined paths.

Developed for the **MCT 332: Design of Mechatronics 2** course at **Ain Shams University**, the robot integrates Computer Vision, Sensor Fusion, and PID control to avoid obstacles and target flames dynamically. The system features a **dual-microcontroller architecture** (ESP32s) communicating with a central computer (Raspberry Pi) via **ROS (Robot Operating System)**.

## 📺 Project Video
https://github.com/user-attachments/assets/92fcacc0-5399-477c-8ba0-baa22fe18960

## 🖼️ Project Poster
<img width="1069" height="1510" alt="image" src="https://github.com/user-attachments/assets/f1b49c92-fe5d-4f02-8c79-77a595bb4f44" />


## 🎨 CAD Renders
<img width="1280" height="720" alt="iso front" src="https://github.com/user-attachments/assets/6b68932d-253a-4155-963e-a58a9fac5052" />
<img width="1280" height="720" alt="topoff" src="https://github.com/user-attachments/assets/117b5b7e-ea44-415e-aafe-43610468fd0f" />
![Back](https://github.com/user-attachments/assets/4dca5c72-1b4b-40b7-86a5-fe9df05ccee8)
![Side](https://github.com/user-attachments/assets/3c1f5489-85d6-44ad-929f-3c0035c6f408)


## ✨ Key Features
* **Fully Autonomous Search:** Executes a search pattern to explore the area until a fire signature is detected.
* **Smart Obstacle Avoidance:** Uses ultrasonic sensors to detect barriers (< 45cm) and autonomously maneuver around them.
* **Hybrid Fire Detection:** * **Long Range:** Computer Vision (Haar Cascades) detects the presence of fire from a distance.
    * **Close Range:** A 5-channel Flame Sensor array provides precise alignment for extinguishing.
* **Active Extinguishing:** Automated water pump activation using a relay system when centered on the target.
* **Remote Dashboard:** A real-time GUI for monitoring camera feeds, sensor states, and robot decisions.

## 🛠 System Architecture

### Hardware Nodes
The system is divided into three main nodes connected via **rosserial**:
1.  **Central Node (Raspberry Pi):** Runs the ROS Master, Computer Vision (`fire_publisher.py`), and Main Logic (`reciever_esp1.py`).
2.  **Sensory Node (ESP32 A):** Handles the 5-channel flame sensor array and 3 ultrasonic sensors.
3.  **Actuation Node (ESP32 B):** Controls the DC motors (with encoders) and the water pump relay.

### Data Flow


[Image of System Block Diagram]

* **Vision Node** publishes `/fire_status` & `/flame_x`.
* **Logic Node** subscribes to sensors, determines the state (Scanning, Obstacle Avoidance, Fire Tracking), and publishes `/cmd_vel`.
* **Motor Node** subscribes to `/cmd_vel` and drives the motors using PID feedback.

## 🔌 Pin Configuration

### ESP32 A (Sensors Node)
| Component | Pin (GPIO) | Function |
| :--- | :--- | :--- |
| **Flame Sensors** | 39, 15, 4, 16, 17 | 5-Channel Array Inputs |
| **Ultrasonic Front** | Trig: 0, Echo: 5 | Distance Measurement |
| **Ultrasonic Left** | Trig: 2, Echo: 18 | Obstacle Detection |
| **Ultrasonic Right** | Trig: 23, Echo: 19 | Obstacle Detection |

### ESP32 B (Actuation Node)
| Component | Pin (GPIO) | Function |
| :--- | :--- | :--- |
| **Right Motor** | PWM: 19 | Speed Control |
| | Dir: 26, 18 | Direction Control |
| | Enc: 34, 35 | Encoder Feedback |
| **Left Motor** | PWM: 32 | Speed Control |
| | Dir: 2, 15 | Direction Control |
| | Enc: 36, 39 | Encoder Feedback |
| **Water Pump** | 25 | Relay Control Signal |

## 🚀 Installation & Setup

### 1. Workspace Setup
```bash
mkdir -p ~/firebot_ws/src
cd ~/firebot_ws/src
catkin_create_pkg dual_rosserial rospy std_msgs sensor_msgs geometry_msgs cv_bridge
# Place all Python scripts inside the /src folder and make them executable
chmod +x *.py
cd ..
catkin_make
source devel/setup.bash

### 2. Microcontroller Firmware
Upload the provided scripts using Arduino IDE:

Sensors: Flash ESP 1.py (C++ code) to ESP32 A.

Motors: Flash ESP 2 motors code.py (C++ code) to ESP32 B.

Note: Ensure ros_lib is configured with the correct IP address of your ROS Master.

### 3. Launching the System
We use a single launch file to start both serial connections and the logic nodes.

Bash
roslaunch dual_rosserial dual_serial.launch
Launch File Breakdown:

Starts 2 rosserial nodes (Port /dev/ttyUSB0 and /dev/ttyUSB1).

Starts fire_publisher.py (Vision).

Starts reciever_esp1.py (Main Logic).

### 4. Running the Dashboard
On a PC connected to the same network as the robot:

Bash
python3 gui.py
Note: Update the WebSocket URI in gui.py to match your Raspberry Pi's IP address (e.g., ws://192.168.1.58:8765).

📊 Logic Description
Scanning Mode: The robot executes a search pattern (Move Forward 3s → Turn) to find a fire.

Obstacle Avoidance: If any Ultrasonic sensor reads < 45cm, the robot interrupts scanning to maneuver away.

Fire Tracking:

Long Range: If the Camera detects fire, the robot uses Vision PID to center the flame horizontally.

Close Range: When Flame Sensors trigger, the robot stops and aligns until the center sensor is active.

Extinguishing: The pump activates for a set duration. The robot verifies the fire is out before sending an email notification and resuming search.
