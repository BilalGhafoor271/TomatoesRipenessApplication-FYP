# 🍅 Tomato Gripper Control System

A computer vision-based tomato detection and robotic gripper control system that prevents tomatoes from falling by implementing slow, controlled movements.

## 🚀 Features

- **Slow Movement Control**: Prevents tomatoes from falling due to fast movements
- **Smart Detection**: Only targets fully-ripe tomatoes with high confidence
- **Smooth Acceleration**: Uses AccelStepper library for smooth motor control
- **Manual Controls**: Web interface for manual gripper control
- **Emergency Stop**: Safety feature to stop all movements immediately
- **Movement Threshold**: Only moves when target position changes significantly

## 🔧 Hardware Requirements

### Arduino Components
- Arduino Uno/Mega
- 3x Stepper Motors (X, Y, Z axes)
- 3x Stepper Motor Drivers (A4988 or similar)
- 1x Servo Motor (for gripper)
- Power supply for motors
- Breadboard and connecting wires

### Pin Connections
```
X-axis: STEP=2, DIR=3
Y-axis: STEP=4, DIR=5  
Z-axis: STEP=6, DIR=7
Gripper Servo: Pin 9
```

## 📋 Software Requirements

### Python Dependencies
```bash
pip install flask ultralytics opencv-python pyserial
```

### Arduino Libraries
Install these libraries in Arduino IDE:
- `AccelStepper` by Mike McCauley
- `Servo` (built-in)

## 🛠️ Setup Instructions

### 1. Arduino Setup
1. Open `tomato_gripper_arduino.ino` in Arduino IDE
2. Install required libraries (AccelStepper, Servo)
3. Connect hardware according to pin definitions
4. Upload code to Arduino
5. Note the COM port (e.g., COM8)

### 2. Python Setup
1. Install Python dependencies:
   ```bash
   pip install flask ultralytics opencv-python pyserial
   ```

2. Update COM port in `app.py`:
   ```python
   arduino = serial.Serial('COM8', 9600, timeout=1)  # Change COM8 to your port
   ```

3. Ensure your trained model file `trained_tomato_yolov8.pt` is in the project directory

### 3. Running the System
1. Start the Python application:
   ```bash
   python app.py
   ```

2. Open web browser and go to: `http://localhost:5000`

3. The system will automatically detect tomatoes and send coordinates to Arduino

## ⚙️ Configuration

### Speed Control (Arduino)
```cpp
const float MAX_SPEED = 500;        // steps per second (lower = slower)
const float ACCELERATION = 200;     // steps per second^2 (lower = smoother)
```

### Detection Parameters (Python)
```python
GRIPPER_MOVEMENT_DELAY = 5.0        # seconds between movements
MIN_CONFIDENCE = 0.8                # minimum detection confidence
MIN_TOMATO_SIZE = 50                # minimum tomato size in pixels
coordinate_threshold = 2.0          # cm - movement threshold
```

## 🎯 How It Prevents Tomato Falling

### 1. **Slow Movement Speed**
- Reduced motor speed from default to 500 steps/second
- Smooth acceleration/deceleration prevents jerky movements

### 2. **Movement Threshold**
- Only moves when target position changes by >2cm
- Prevents unnecessary small movements

### 3. **Increased Detection Confidence**
- Higher confidence threshold (0.8) ensures only clear detections
- Larger minimum tomato size prevents false positives

### 4. **Longer Movement Intervals**
- 5-second delay between movements (increased from 3 seconds)
- Gives Arduino time to complete movements

### 5. **Safe Distance Control**
- Minimum Z distance of 5cm prevents collision
- Proper gripper open/close timing

## 🚨 Troubleshooting

### Common Issues

1. **Arduino Not Connecting**
   - Check COM port in Device Manager
   - Update port in `app.py`
   - Ensure Arduino is powered and connected

2. **Tomatoes Still Falling**
   - Reduce `MAX_SPEED` in Arduino code (try 300 or 200)
   - Increase `GRIPPER_MOVEMENT_DELAY` to 7-10 seconds
   - Check motor connections and power supply

3. **Poor Detection**
   - Adjust lighting conditions
   - Retrain model with more diverse data
   - Lower `MIN_CONFIDENCE` temporarily for testing

4. **Motors Not Moving**
   - Check stepper driver connections
   - Verify power supply voltage
   - Test individual motors with Arduino IDE examples

### Emergency Procedures
- Use Emergency Stop button on web interface
- Disconnect power supply immediately if needed
- Check for loose connections or damaged components

## 📊 Performance Optimization

### For Better Tomato Handling
1. **Reduce Speed Further**: Set `MAX_SPEED = 300` for very delicate tomatoes
2. **Increase Movement Delay**: Set `GRIPPER_MOVEMENT_DELAY = 8.0` for more time
3. **Adjust Gripper Pressure**: Modify servo angles in Arduino code
4. **Add Vibration Damping**: Use rubber mounts for motors

### For Faster Operation
1. **Increase Speed**: Set `MAX_SPEED = 800` (test carefully)
2. **Reduce Movement Delay**: Set `GRIPPER_MOVEMENT_DELAY = 3.0`
3. **Lower Detection Threshold**: Set `MIN_CONFIDENCE = 0.7`

## 🔒 Safety Features

- Emergency stop functionality
- Movement completion verification
- Safe distance limits
- Error handling for serial communication
- Manual override controls

## 📝 License

This project is open source. Feel free to modify and improve for your specific needs.

## 🤝 Contributing

If you find issues or have improvements, please:
1. Test thoroughly before reporting
2. Provide detailed error messages
3. Include hardware setup details
4. Suggest specific improvements 