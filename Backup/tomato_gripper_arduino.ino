#include <Servo.h>
#include <AccelStepper.h>

// Pin definitions
#define X_STEP_PIN 2
#define X_DIR_PIN 3
#define Y_STEP_PIN 4
#define Y_DIR_PIN 5
#define Z_STEP_PIN 6
#define Z_DIR_PIN 7

#define GRIPPER_SERVO_PIN 9
#define GRIPPER_OPEN_ANGLE 90
#define GRIPPER_CLOSE_ANGLE 30

// Stepper motor objects
AccelStepper stepperX(1, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(1, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(1, Z_STEP_PIN, Z_DIR_PIN);

// Servo for gripper
Servo gripperServo;

// Movement parameters
float currentX = 0;
float currentY = 0;
float currentZ = 0;
float targetX = 0;
float targetY = 0;
float targetZ = 0;

// Speed control (lower = slower, safer for tomatoes)
const float MAX_SPEED = 500;  // steps per second
const float ACCELERATION = 200; // steps per second^2
const int MOVEMENT_DELAY = 100; // ms between movements

// Gripper states
bool gripperOpen = true;
bool isMoving = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize stepper motors
  stepperX.setMaxSpeed(MAX_SPEED);
  stepperX.setAcceleration(ACCELERATION);
  
  stepperY.setMaxSpeed(MAX_SPEED);
  stepperY.setAcceleration(ACCELERATION);
  
  stepperZ.setMaxSpeed(MAX_SPEED);
  stepperZ.setAcceleration(ACCELERATION);
  
  // Initialize gripper servo
  gripperServo.attach(GRIPPER_SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  
  Serial.println("Tomato Gripper System Ready");
  Serial.println("Format: x,y,z (e.g., 1.5,2.0,10.0)");
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.length() > 0) {
      parseAndExecuteCommand(data);
    }
  }
  
  // Run stepper motors
  stepperX.run();
  stepperY.run();
  stepperZ.run();
  
  // Check if movement is complete
  if (isMoving && !stepperX.isRunning() && !stepperY.isRunning() && !stepperZ.isRunning()) {
    isMoving = false;
    Serial.println("Movement complete");
    
    // Close gripper after reaching target
    if (!gripperOpen) {
      closeGripper();
      delay(1000); // Hold for 1 second
      openGripper();
    }
  }
}

void parseAndExecuteCommand(String command) {
  // Parse x,y,z coordinates
  int comma1 = command.indexOf(',');
  int comma2 = command.indexOf(',', comma1 + 1);
  
  if (comma1 != -1 && comma2 != -1) {
    float x = command.substring(0, comma1).toFloat();
    float y = command.substring(comma1 + 1, comma2).toFloat();
    float z = command.substring(comma2 + 1).toFloat();
    
    Serial.print("Received coordinates: X=");
    Serial.print(x);
    Serial.print(" Y=");
    Serial.print(y);
    Serial.print(" Z=");
    Serial.println(z);
    
    // Move to target position
    moveToPosition(x, y, z);
  }
}

void moveToPosition(float x, float y, float z) {
  // Calculate steps needed (convert cm to steps)
  // Adjust these conversion factors based on your hardware
  const float CM_TO_STEPS_X = 100; // steps per cm
  const float CM_TO_STEPS_Y = 100;
  const float CM_TO_STEPS_Z = 100;
  
  long stepsX = (x - currentX) * CM_TO_STEPS_X;
  long stepsY = (y - currentY) * CM_TO_STEPS_Y;
  long stepsZ = (z - currentZ) * CM_TO_STEPS_Z;
  
  // Set target positions
  stepperX.moveTo(stepperX.currentPosition() + stepsX);
  stepperY.moveTo(stepperY.currentPosition() + stepsY);
  stepperZ.moveTo(stepperZ.currentPosition() + stepsZ);
  
  // Update current position
  currentX = x;
  currentY = y;
  currentZ = z;
  
  isMoving = true;
  
  Serial.println("Starting movement...");
}

void openGripper() {
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  gripperOpen = true;
  Serial.println("Gripper opened");
}

void closeGripper() {
  gripperServo.write(GRIPPER_CLOSE_ANGLE);
  gripperOpen = false;
  Serial.println("Gripper closed");
}

// Emergency stop function
void emergencyStop() {
  stepperX.stop();
  stepperY.stop();
  stepperZ.stop();
  stepperX.setCurrentPosition(stepperX.currentPosition());
  stepperY.setCurrentPosition(stepperY.currentPosition());
  stepperZ.setCurrentPosition(stepperZ.currentPosition());
  isMoving = false;
  Serial.println("Emergency stop activated");
} 