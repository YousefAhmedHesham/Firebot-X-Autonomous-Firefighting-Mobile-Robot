import tkinter as tk
from PIL import Image, ImageTk
import asyncio
import websockets
import threading
import re
from io import BytesIO

def update_gui(message):
    # Print raw message for debugging
    print(f"Received message: {message[:100]}...")
    
    # Process Action Status with specific formatting
    if "ACTION:" in message:
        action = message.split("ACTION:")[-1].strip()
        
        # Handle "Moving forward" case
        if "Moving forward" in action:
            action_label.config(text="Moving forward (clear path)", fg="green", font=("Helvetica", 12, "bold"))
        
        # Handle "Obstacle detected" case
        elif "Obstacle detected" in action:
            # Get sensor readings from message
            if "Sensor Readings:" in message:
                readings_part = message.split("Sensor Readings:")[-1].strip()
                readings = [x.strip() for x in readings_part[1:-1].split(',')]
                try:
                    front = float(readings[0].split(':')[1].strip().replace(' cm', ''))
                    left = float(readings[1].split(':')[1].strip().replace(' cm', ''))
                    right = float(readings[2].split(':')[1].strip().replace(' cm', ''))
                    
                    if left < right:
                        action_text = "Obstacle detected → Turning left"
                    else:
                        action_text = "Obstacle detected → Turning right"
                    
                    action_label.config(text=action_text, fg="orange", font=("Helvetica", 12, "bold"))
                except Exception as e:
                    print(f"Error processing sensor data: {e}")
                    action_label.config(text="Obstacle detected", fg="orange", font=("Helvetica", 12, "bold"))
            else:
                action_label.config(text="Obstacle detected", fg="orange", font=("Helvetica", 12, "bold"))
        
        # Default case for other actions
        else:
            action_label.config(text=action, fg="green", font=("Helvetica", 12, "bold"))
    
    # Process Pump Status
    if "Pump Status:" in message:
        status_part = message.split("Pump Status:")[-1].strip()
        pump_status = status_part.split()[0].strip().upper()
        if pump_status == "ON":
            pump_label.config(text="💧 PUMP ACTIVE 💧", fg="blue", font=("Helvetica", 14, "bold"))
        else:
            pump_label.config(text="Pump: INACTIVE", fg="black", font=("Helvetica", 14))
    
    # Process Sensor Readings (for display purposes)
    if "Sensor Readings:" in message or "=== Sensor Readings ===" in message:
        if "=== Sensor Readings ===" in message:
            lines = [line.strip() for line in message.split('\n') if line.strip()]
            front = lines[1].split(':')[-1].strip().replace("-1.0 cm", "No Obstacle")
            left = lines[2].split(':')[-1].strip().replace("-1.0 cm", "No Obstacle")
            right = lines[3].split(':')[-1].strip().replace("-1.0 cm", "No Obstacle")
        else:
            readings = message.split("Sensor Readings:")[-1].strip()
            readings = readings.replace("-1.0", "No Obstacle")
            parts = [p.strip() for p in readings[1:-1].split(',')]
            front = parts[0].split(':')[-1].strip()
            left = parts[1].split(':')[-1].strip()
            right = parts[2].split(':')[-1].strip()
        
        sensor_readings_label.config(text=f"Front: {front}\nLeft: {left}\nRight: {right}")
    
    # Process Flame Sensor Readings
    if "Flame Sensor Readings:" in message:
        match = re.search(r'\((\d+,\s*\d+,\s*\d+,\s*\d+,\s*\d+)\)', message)
        if match:
            try:
                readings = eval(match.group(0))
                flame_text = ""
                for i, val in enumerate(readings, 1):
                    status = "🔥" if val == 1 else "❄️"
                    flame_text += f"Sensor {i}: {status}\n"
                flame_readings_label.config(text=flame_text)
                
                if 1 in readings:
                    fire_status_label.config(text="FIRE DETECTED! 🔥", fg="red")
                else:
                    fire_status_label.config(text="No Fire Detected ❄️", fg="blue")
            except Exception as e:
                print(f"Error processing flame sensors: {e}")

def update_camera_feed(image_data):
    try:
        image = Image.open(BytesIO(image_data))
        image = image.resize((320, 240), Image.LANCZOS)
        photo = ImageTk.PhotoImage(image)
        camera_feed_label.config(image=photo, text="")
        camera_feed_label.image = photo
    except Exception as e:
        print(f"Camera feed error: {e}")

# Create main window
root = tk.Tk()
root.title("Firebot X Dashboard")
root.geometry("800x600")
root.resizable(False, False)

# Background
try:
    bg_img = Image.open("D:/University/Spring 2025 Senior 1/Design of Mechatronic Systems (2)/GUI/fire_background.png")
    bg_img = bg_img.resize((800, 600), Image.LANCZOS)
    bg_photo = ImageTk.PhotoImage(bg_img)
    bg_label = tk.Label(root, image=bg_photo)
    bg_label.place(x=0, y=0, relwidth=1, relheight=1)
except Exception as e:
    print(f"Background error: {e}")
    root.configure(bg="white")

# Title
title_label = tk.Label(root, text="FIREBOT X DASHBOARD", 
                      font=("Helvetica", 20, "bold"), bg="white")
title_label.pack(pady=10)

# Status Bar
status_frame = tk.Frame(root, bg="white")
status_frame.pack(pady=5)

fire_status_label = tk.Label(status_frame, text="No Fire Detected ❄️", 
                           font=("Helvetica", 14, "bold"), fg="blue", bg="white")
fire_status_label.pack(side=tk.LEFT, padx=20)

pump_label = tk.Label(status_frame, text="Pump: INACTIVE", 
                     font=("Helvetica", 14), bg="white")
pump_label.pack(side=tk.LEFT, padx=20)

# Sensors Frame
sensors_frame = tk.Frame(root, bg="white")
sensors_frame.pack(pady=10)

# Distance Sensors
distance_frame = tk.Frame(sensors_frame, bg="white")
distance_frame.pack(side=tk.LEFT, padx=40)

tk.Label(distance_frame, text="DISTANCE SENSORS", 
        font=("Helvetica", 12, "bold"), bg="white").pack()
sensor_readings_label = tk.Label(distance_frame, 
                               text="Front: -\nLeft: -\nRight: -",
                               font=("Helvetica", 12), 
                               bg="white", 
                               justify=tk.LEFT)
sensor_readings_label.pack()

# Flame Sensors
flame_frame = tk.Frame(sensors_frame, bg="white")
flame_frame.pack(side=tk.LEFT, padx=40)

tk.Label(flame_frame, text="FLAME SENSORS", 
        font=("Helvetica", 12, "bold"), bg="white").pack()
flame_readings_label = tk.Label(flame_frame, 
                              text="Sensor 1: ❄️\nSensor 2: ❄️\nSensor 3: ❄️\nSensor 4: ❄️\nSensor 5: ❄️",
                              font=("Helvetica", 12), 
                              bg="white", 
                              justify=tk.LEFT)
flame_readings_label.pack()

# Camera Feed
camera_feed_label = tk.Label(root, text="Camera Feed Loading...", bg="white")
camera_feed_label.pack(pady=20)

# Action Display
action_frame = tk.Frame(root, bg="white")
action_frame.pack(pady=5)
tk.Label(action_frame, text="Action:", 
        font=("Helvetica", 12), bg="white").pack(side=tk.LEFT)
action_label = tk.Label(action_frame, text="Waiting...", 
                       font=("Helvetica", 12, "bold"), 
                       bg="white", fg="green",
                       width=40)
action_label.pack(side=tk.LEFT)

# WebSocket Client
async def receive_only():
    uri = "ws://192.168.1.58:8765"
    async with websockets.connect(uri) as websocket:
        while True:
            response = await websocket.recv()
            if isinstance(response, bytes):
                root.after(0, update_camera_feed, response)
            else:
                root.after(0, update_gui, response)

def start_async_task():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(receive_only())

threading.Thread(target=start_async_task, daemon=True).start()
root.mainloop()