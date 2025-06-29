from flask import Flask, render_template, Response
from ultralytics import YOLO
import cv2
import serial
import time
import threading

app = Flask(__name__)
model = YOLO("trained_tomato_yolov8.pt")

# Arduino Serial Port
try:
    arduino = serial.Serial('COM8', 9600, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected on COM8")
except Exception as e:
    print("❌ Error opening serial port:", e)
    arduino = None

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
KNOWN_WIDTH_CM = 4
FOCAL_LENGTH = 500

# Gripper control parameters
GRIPPER_MOVEMENT_DELAY = 5.0  # seconds between movements (increased from 3)
MIN_CONFIDENCE = 0.8  # Increased confidence threshold
MIN_TOMATO_SIZE = 50  # Increased minimum size for better detection

# Movement smoothing
last_sent_coordinates = None
coordinate_threshold = 2.0  # cm - only move if target is significantly different

def estimate_distance(pixel_width):
    return (KNOWN_WIDTH_CM * FOCAL_LENGTH) / pixel_width

def send_to_arduino_safe(msg):
    """Safely send command to Arduino with error handling"""
    if arduino and arduino.is_open:
        try:
            arduino.write(msg.encode())
            print(f"✅ Sent to Arduino: {msg.strip()}")
            return True
        except Exception as e:
            print(f"❌ Failed to send to Arduino: {e}")
            return False
    return False

def should_move_to_target(new_coords, last_coords):
    """Check if movement is necessary based on distance threshold"""
    if last_coords is None:
        return True
    
    x_diff = abs(new_coords[0] - last_coords[0])
    y_diff = abs(new_coords[1] - last_coords[1])
    z_diff = abs(new_coords[2] - last_coords[2])
    
    # Only move if significant change in position
    return (x_diff > coordinate_threshold or 
            y_diff > coordinate_threshold or 
            z_diff > coordinate_threshold)

def generate():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("❌ Could not open camera.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    last_sent_time = time.time()
    global last_sent_coordinates

    while True:
        success, frame = cap.read()
        if not success:
            break

        results = model(frame)[0]
        best_box = None
        max_area = 0

        # Improved detection parameters
        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            if model.names[cls].lower() != 'fully-ripe':
                continue
            if conf < MIN_CONFIDENCE:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            w = x2 - x1
            h = y2 - y1

            if w < MIN_TOMATO_SIZE or h < MIN_TOMATO_SIZE:
                continue

            area = w * h
            if area > max_area:
                best_box = (x1, y1, x2, y2)
                max_area = area

        # Improved movement logic
        current_time = time.time()
        if (best_box and 
            (current_time - last_sent_time) > GRIPPER_MOVEMENT_DELAY):
            
            x1, y1, x2, y2 = best_box
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            width = x2 - x1
            distance = estimate_distance(width)

            # Calculate target coordinates
            x_pos = (cx - CAMERA_WIDTH // 2) * 0.1
            y_pos = (CAMERA_HEIGHT - cy) * 0.1
            z_pos = max(distance, 5.0)  # Minimum safe distance

            new_coordinates = (x_pos, y_pos, z_pos)
            
            # Check if movement is necessary
            if should_move_to_target(new_coordinates, last_sent_coordinates):
                msg = f"{x_pos:.2f},{y_pos:.2f},{z_pos:.2f}\n"
                print(f"[Sending to Arduino]: {msg.strip()}")
                
                if send_to_arduino_safe(msg):
                    last_sent_time = current_time
                    last_sent_coordinates = new_coordinates
                    
                    # Add delay to allow Arduino to process
                    time.sleep(0.5)

            # Draw detection box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"fully-ripe {conf:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            
            # Display coordinates
            cv2.putText(frame, f"X:{x_pos:.1f} Y:{y_pos:.1f} Z:{z_pos:.1f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/emergency_stop')
def emergency_stop():
    """Emergency stop command"""
    if arduino and arduino.is_open:
        arduino.write(b"STOP\n")
        return {"status": "success", "message": "Emergency stop activated"}
    return {"status": "error", "message": "Arduino not connected"}

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
