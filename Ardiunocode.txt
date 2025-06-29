#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 100
#define SERVO_MAX 500

// Servo channels
#define BASE        0
#define SHOULDER    1
#define ELBOW       2
#define WRIST1      3       // wrist pitch
#define WRIST_ROT   4       // wrist rotation
#define GRIPPER     5       // gripper

// Safe angle limits
int minAngles[6] = {15, 20, 30, 10, 0, 40};
int maxAngles[6] = {165, 150, 140, 170, 180, 120};

// Arm link lengths (cm)
const float L1 = 10.0;
const float L2 = 15.0;
const float L3 = 15.0;

void moveServo(int channel, int angle) {
  angle = constrain(angle, minAngles[channel], maxAngles[channel]);
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Inverse Kinematics
bool inverseKinematics(float x, float y, float z,
                       int &baseAngle, int &shoulderAngle, int &elbowAngle,
                       int &wrist1Angle, int &wristRotAngle) {

  baseAngle = (int)(atan2(y, x) * 180.0 / M_PI);
  baseAngle = constrain(baseAngle, minAngles[BASE], maxAngles[BASE]);

  float r = sqrt(x * x + y * y);
  float wristX = r;
  float wristZ = z - L1;

  float dist = sqrt(wristX * wristX + wristZ * wristZ);
  if (dist > (L2 + L3)) return false;

  float cosElbow = (dist * dist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  if (cosElbow < -1.0 || cosElbow > 1.0) return false;

  float elbowRad = acos(cosElbow);
  elbowAngle = (int)(elbowRad * 180.0 / M_PI);
  elbowAngle = constrain(elbowAngle, minAngles[ELBOW], maxAngles[ELBOW]);

  float k1 = L2 + L3 * cos(elbowRad);
  float k2 = L3 * sin(elbowRad);
  float shoulderRad = atan2(wristZ, wristX) - atan2(k2, k1);
  shoulderAngle = (int)(shoulderRad * 180.0 / M_PI);
  shoulderAngle = constrain(shoulderAngle, minAngles[SHOULDER], maxAngles[SHOULDER]);

  wrist1Angle = 180 - elbowAngle - shoulderAngle;
  wrist1Angle = constrain(wrist1Angle, minAngles[WRIST1], maxAngles[WRIST1]);

  wristRotAngle = 90;  // fixed or later configurable
  wristRotAngle = constrain(wristRotAngle, minAngles[WRIST_ROT], maxAngles[WRIST_ROT]);

  return true;
}

// Full Arm Movement
void moveToPosition(float x, float y, float z) {
  int baseAngle, shoulderAngle, elbowAngle, wrist1Angle, wristRotAngle;
  bool ok = inverseKinematics(x, y, z,
                               baseAngle, shoulderAngle, elbowAngle,
                               wrist1Angle, wristRotAngle);

  if (!ok) {
    Serial.println("Target unreachable");
    return;
  }

  moveServo(BASE, baseAngle);
  moveServo(SHOULDER, shoulderAngle);
  moveServo(ELBOW, elbowAngle);
  moveServo(WRIST1, wrist1Angle);
  moveServo(WRIST_ROT, wristRotAngle);
  delay(1000);
}

// Gripper controls
void openGripper() {
  Serial.println("Opening gripper");
  moveServo(GRIPPER, 50);
  delay(700);
}

void closeGripper() {
  Serial.println("Closing gripper");
  moveServo(GRIPPER, 100);
  delay(1000);
}

void pickObject() {
  closeGripper();
}

void releaseObject() {
  openGripper();
}

void moveHome() {
  Serial.println("Moving to home");
  moveServo(BASE, 90);
  moveServo(SHOULDER, 20);
  moveServo(ELBOW, 120);
  moveServo(WRIST1, 70);
  moveServo(WRIST_ROT, 90);
  openGripper();  // Start open instead of closed
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
  moveHome();
  Serial.println("6DOF Robotic Arm Ready");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

     if (input == "STOP") {
      Serial.println("Emergency stop received!");
      moveHome();  // Optionally move to safe position
      return;      // Skip further execution
      }

    int firstComma = input.indexOf(',');
    int lastComma = input.lastIndexOf(',');

    if (firstComma == -1 || lastComma == -1 || firstComma == lastComma) {
      Serial.println("Invalid input format");
      return;
    }

    String xs = input.substring(0, firstComma);
    String ys = input.substring(firstComma + 1, lastComma);
    String zs = input.substring(lastComma + 1);

    xs.trim(); ys.trim(); zs.trim();

    float x = xs.toFloat();
    float y = ys.toFloat();
    float z = zs.toFloat();

    if ((x == 0 && xs != "0" && xs != "0.0") ||
        (y == 0 && ys != "0" && ys != "0.0") ||
        (z == 0 && zs != "0" && zs != "0.0")) {
      Serial.println("Invalid numeric values");
      return;
    }

    Serial.print("Moving to: ");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.println(z);

    moveToPosition(x, y, z);
    delay(1000);
    pickObject();

    float binX = 15.0, binY = 0.0, binZ = 30.0;
    moveToPosition(binX, binY, binZ);
    delay(500);
    releaseObject();

    moveHome();
    Serial.println("Done");
  } 
}
