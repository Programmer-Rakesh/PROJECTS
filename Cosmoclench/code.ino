#include <Servo.h>

// === Joystick Pins ===
#define JOY1_X A0
#define JOY1_Y A1
#define JOY2_X A2
#define JOY2_Y A3

// === L298N Motor Driver Pins ===
// Left Driver (Motors A1, A2)
#define A1_IN1 6
#define A1_IN2 7
#define A2_IN1 8
#define A2_IN2 9

// Right Driver (Motors B1, B2)
#define B1_IN1 10
#define B1_IN2 11
#define B2_IN1 12
#define B2_IN2 13

// === Servo Pins ===
#define GRIPPER_SERVO1_PIN 5
#define GRIPPER_SERVO2_PIN 4
#define ARM_SERVO_PIN 3

Servo gripperServo1;
Servo gripperServo2;
Servo armServo;

void setup() {
  Serial.begin(9600);

  // Motor pin setup
  pinMode(A1_IN1, OUTPUT); pinMode(A1_IN2, OUTPUT);
  pinMode(A2_IN1, OUTPUT); pinMode(A2_IN2, OUTPUT);
  pinMode(B1_IN1, OUTPUT); pinMode(B1_IN2, OUTPUT);
  pinMode(B2_IN1, OUTPUT); pinMode(B2_IN2, OUTPUT);

  // Attach servos
  gripperServo1.attach(GRIPPER_SERVO1_PIN);
  gripperServo2.attach(GRIPPER_SERVO2_PIN);
  armServo.attach(ARM_SERVO_PIN);
}

void loop() {
  int leftDir, rightDir;

  // --- Read joystick 1 for drive ---
  mapJoystickToMotors(leftDir, rightDir);

  // --- Read joystick 2 for servos ---
  int gripperPos = mapJoystickToServo(JOY2_X);    // 0–180°
  int armPos = mapJoystickToArmServo(JOY2_Y);     // 0–90°

  // --- Control 4 DC motors (no EN pins) ---
  controlMotorPair(A1_IN1, A1_IN2, A2_IN1, A2_IN2, leftDir);
  controlMotorPair(B1_IN1, B1_IN2, B2_IN1, B2_IN2, rightDir);

  // --- Control servos ---
  gripperServo1.write(gripperPos);
  gripperServo2.write(180 - gripperPos); // mirror motion
  armServo.write(armPos);

  // --- Debug info ---
  printStatus(leftDir, rightDir, gripperPos, armPos);

  delay(20);
}

// === Map joystick to motor direction ===
// joyY controls forward/back; joyX controls turn
void mapJoystickToMotors(int &leftDir, int &rightDir) {
  int joyX = analogRead(JOY1_X) - 512;
  int joyY = analogRead(JOY1_Y) - 512;

  int leftVal  = joyY - joyX;
  int rightVal = joyY + joyX;

  // Determine direction: -1 reverse, 0 stop, 1 forward
  leftDir  = (abs(leftVal) < 100) ? 0 : (leftVal > 0 ? 1 : -1);
  rightDir = (abs(rightVal) < 100) ? 0 : (rightVal > 0 ? 1 : -1);
}

// === Map joystick to full servo (0–180°) ===
int mapJoystickToServo(int pin) {
  int raw = analogRead(pin);
  return map(raw, 0, 1023, 0, 180);
}

// === Map joystick to arm servo (0–90°) ===
int mapJoystickToArmServo(int pin) {
  int raw = analogRead(pin);
  return map(raw, 0, 1023, 0, 90);
}

// === Control a pair of motors (same side) ===
void controlMotorPair(int in1a, int in2a, int in1b, int in2b, int dir) {
  if (dir > 0) { // forward
    digitalWrite(in1a, HIGH); digitalWrite(in2a, LOW);
    digitalWrite(in1b, HIGH); digitalWrite(in2b, LOW);
  } else if (dir < 0) { // reverse
    digitalWrite(in1a, LOW); digitalWrite(in2a, HIGH);
    digitalWrite(in1b, LOW); digitalWrite(in2b, HIGH);
  } else { // stop
    digitalWrite(in1a, LOW); digitalWrite(in2a, LOW);
    digitalWrite(in1b, LOW); digitalWrite(in2b, LOW);
  }
}

// === Debug ===
void printStatus(int left, int right, int grip, int arm) {
  Serial.print("Left Dir: "); Serial.print(left);
  Serial.print(" | Right Dir: "); Serial.print(right);
  Serial.print(" | Gripper: "); Serial.print(grip);
  Serial.print(" | Arm: "); Serial.println(arm);
}