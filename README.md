# 🍅 Farm Hand Robotic Arm - Tomato Detection & Gripping System

A sophisticated computer vision-based tomato detection and 6DOF robotic arm control system designed for gentle, precise tomato harvesting with fall prevention mechanisms.

## 🚀 Features

- **6DOF Robotic Arm Control**: Full 6-degree-of-freedom arm with inverse kinematics
- **Computer Vision Detection**: YOLOv8-based tomato detection with confidence filtering
- **Gentle Movement Control**: Slow, controlled movements to prevent tomato damage
- **Real-time Web Interface**: Live camera feed with emergency controls
- **Inverse Kinematics**: Automatic coordinate conversion for precise positioning
- **Safety Features**: Emergency stop, movement thresholds, and safe distance limits
- **Smart Detection**: Only targets fully-ripe tomatoes with high confidence
- **Automatic Pick & Place**: Complete harvesting cycle with bin placement

## 🔧 Hardware Requirements

### 6DOF Robotic Arm Components
- **Base Servo**: 180° rotation for arm positioning
- **Shoulder Servo**: 130° range for vertical movement
- **Elbow Servo**: 110° range for arm extension
- **Wrist Pitch Servo**: 160° range for wrist orientation
- **Wrist Rotation Servo**: 180° range for tool orientation
- **Gripper Servo**: 80° range for opening/closing
- **PCA9685 PWM Controller**: For servo control
- **Arduino Uno/Mega**: Main controller

### Alternative Stepper Motor Setup (Backup)
- 3x Stepper Motors (X, Y, Z axes)
- 3x Stepper Motor Drivers (A4988 or similar)
- 1x Servo Motor (for gripper)
- Power supply for motors

### Camera System
- USB Camera (640x480 resolution)
- Computer with Python support

## 📋 Software Requirements

### Python Dependencies
```bash
pip install flask ultralytics opencv-python pyserial
```

### Arduino Libraries
Install these libraries in Arduino IDE:
- `Adafruit PWM Servo Driver Library` (for 6DOF arm)
- `Wire` (built-in)
- `Servo` (built-in)
- `AccelStepper` (for stepper motor version)

## 🛠️ Setup Instructions

### 1. Arduino Setup (6DOF Arm)
1. Open `ardiuno.ino` in Arduino IDE
2. Install required libraries:
   - Adafruit PWM Servo Driver Library
   - Wire (built-in)
3. Connect hardware according to servo channel definitions:
   ```
   Channel 0: Base Servo
   Channel 1: Shoulder Servo
   Channel 2: Elbow Servo
   Channel 3: Wrist Pitch Servo
   Channel 4: Wrist Rotation Servo
   Channel 5: Gripper Servo
   ```
4. Upload code to Arduino
5. Note the COM port (e.g., COM8)

### 2. Alternative Stepper Setup
1. Use `Backup/tomato_gripper_arduino.ino` for stepper motor version
2. Connect stepper motors:
   ```
   X-axis: STEP=2, DIR=3
   Y-axis: STEP=4, DIR=5  
   Z-axis: STEP=6, DIR=7
   Gripper Servo: Pin 9
   ```

### 3. Python Setup
1. Install Python dependencies:
   ```bash
   pip install flask ultralytics opencv-python pyserial
   ```

2. Update COM port in `app.py`:
   ```python
   arduino = serial.Serial('COM8', 9600, timeout=1)  # Change COM8 to your port
   ```

3. Ensure your trained model file `trained_tomato_yolov8.pt` is in the project directory

### 4. Running the System
1. Start the Python application:
   ```bash
   python app.py
   ```

2. Open web browser and go to: `http://localhost:5000`

3. The system will automatically detect tomatoes and send coordinates to Arduino

## ⚙️ Configuration

### 6DOF Arm Parameters (Arduino)
```cpp
// Arm link lengths (cm)
const float L1 = 10.0;  // Base to shoulder
const float L2 = 15.0;  // Shoulder to elbow
const float L3 = 15.0;  // Elbow to wrist

// Safe angle limits
int minAngles[6] = {15, 20, 30, 10, 0, 40};
int maxAngles[6] = {165, 150, 140, 170, 180, 120};
```

### Stepper Motor Parameters (Backup)
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
- Reduced motor speed and smooth acceleration
- 5-second delay between movements
- Gentle servo movements with proper delays

### 2. **Movement Threshold**
- Only moves when target position changes by >2cm
- Prevents unnecessary small movements

### 3. **High Detection Confidence**
- 0.8 confidence threshold ensures only clear detections
- Larger minimum tomato size prevents false positives

### 4. **Safe Distance Control**
- Minimum Z distance of 5cm prevents collision
- Proper gripper open/close timing

### 5. **Inverse Kinematics**
- Smooth trajectory planning
- Constrained movement within safe angles

## 🔄 System Workflow

### 1. **Detection Phase**
- Camera captures live feed
- YOLOv8 model detects fully-ripe tomatoes
- Filters by confidence and size

### 2. **Coordinate Calculation**
- Calculates center point of detected tomato
- Converts pixel coordinates to real-world coordinates
- Estimates distance using known object width

### 3. **Movement Planning**
- Checks if movement is necessary (threshold)
- Sends coordinates to Arduino
- Arduino calculates inverse kinematics

### 4. **Harvesting Cycle**
- Move to tomato position
- Close gripper
- Move to bin position
- Release tomato
- Return to home position

## 🚨 Troubleshooting

### Common Issues

1. **Arduino Not Connecting**
   - Check COM port in Device Manager
   - Update port in `app.py`
   - Ensure Arduino is powered and connected

2. **Servos Not Moving**
   - Check PCA9685 connections
   - Verify power supply voltage
   - Test individual servos with Arduino IDE

3. **Poor Detection**
   - Adjust lighting conditions
   - Retrain model with more diverse data
   - Lower `MIN_CONFIDENCE` temporarily for testing

4. **Arm Not Reaching Target**
   - Check arm link lengths in code
   - Verify angle limits
   - Ensure target is within reach

### Emergency Procedures
- Use Emergency Stop button on web interface
- Disconnect power supply immediately if needed
- Check for loose connections or damaged components

## 📊 Performance Optimization

### For Better Tomato Handling
1. **Reduce Movement Speed**: Increase delays between movements
2. **Adjust Gripper Pressure**: Modify servo angles in Arduino code
3. **Fine-tune Detection**: Adjust confidence and size thresholds
4. **Add Vibration Damping**: Use rubber mounts for servos

### For Faster Operation
1. **Increase Movement Speed**: Reduce delays (test carefully)
2. **Lower Detection Threshold**: Set `MIN_CONFIDENCE = 0.7`
3. **Optimize Arm Trajectories**: Adjust inverse kinematics parameters

## 🔒 Safety Features

- Emergency stop functionality
- Movement completion verification
- Safe distance limits
- Error handling for serial communication
- Manual override controls
- Angle constraints for all servos

## 📁 Project Structure

```
bilal - Copy/
├── app.py                          # Main Flask application
├── ardiuno.ino                     # 6DOF robotic arm controller
├── templates/
│   └── index.html                  # Web interface
├── static/uploads/                 # Sample images
├── trained_tomato_yolov8.pt        # YOLOv8 trained model
├── Model_training.ipynb            # Model training notebook
├── Backup/
│   ├── tomato_gripper_arduino.ino  # Stepper motor version
│   ├── ee.txt                      # Additional documentation
│   └── gg.py                       # Utility scripts
└── README.md                       # This file
```

## 🎓 Model Training

The system uses a custom-trained YOLOv8 model for tomato detection. The training process is documented in `Model_training.ipynb`. The model is specifically trained to detect fully-ripe tomatoes with high accuracy.

## 📝 License

This project is open source. Feel free to modify and improve for your specific needs.

## 🤝 Contributing

If you find issues or have improvements, please:
1. Test thoroughly before reporting
2. Provide detailed error messages
3. Include hardware setup details
4. Suggest specific improvements 