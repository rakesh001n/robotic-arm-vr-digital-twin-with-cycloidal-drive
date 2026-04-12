#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// =====================
// PCA9685 SETUP
// =====================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// Servo pulse limits (TUNE THIS!)
#define SERVOMIN 120
#define SERVOMAX 520

// Servo channels
#define SHOULDER_CH 0
#define ELBOW_CH 1
#define WRIST_CH 2
#define WRIST_ROLL_CH 3
#define GRIPPER_CH 4

// =====================
// JOINT LIMITS
// =====================
#define SHOULDER_MIN 40
#define SHOULDER_MAX 130

#define ELBOW_MIN 40
#define ELBOW_MAX 130

#define WRIST_MIN 40
#define WRIST_MAX 130

// =====================
// STEPPER (A4988)
// =====================
#define STEP_PIN 33
#define DIR_PIN 25
#define ENABLE_PIN 32

#define STEPS_PER_REV 200   // adjust based on microstepping

float currentBaseAngle = 0.0;

// =====================
// SERIAL PARSER
// =====================
String buffer = "";
bool receiving = false;

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);

  // PCA9685 init
  pca.begin();
  pca.setPWMFreq(50);  // servo frequency

  // Stepper setup
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  Serial.println("ESP32 Ready (Servo + Stepper)");
}

// =====================
// MAIN LOOP
// =====================
void loop() {
  readSerial();
}

// =====================
// SERIAL READ (<...>)
// =====================
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '<') {
      buffer = "";
      receiving = true;
    }
    else if (c == '>') {
      receiving = false;
      processData(buffer);
    }
    else if (receiving) {
      buffer += c;
    }
  }
}

// =====================
// ANGLE → PWM
// =====================
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// =====================
// PROCESS DATA
// =====================
void processData(String data) {

  int values[6] = {0};
  int index = 0;

  char temp[data.length() + 1];
  data.toCharArray(temp, sizeof(temp));

  char *token = strtok(temp, ",");

  while (token != NULL && index < 6) {
    values[index++] = atoi(token);
    token = strtok(NULL, ",");
  }

  if (index < 6) {
    Serial.println("Invalid data");
    return;
  }

  // =====================
  // APPLY JOINT LIMITS
  // =====================
  int shoulder = constrain(values[0], SHOULDER_MIN, SHOULDER_MAX);
  int elbow    = constrain(values[1], ELBOW_MIN, ELBOW_MAX);
  int wrist    = constrain(values[2], WRIST_MIN, WRIST_MAX);

  int wristRoll = constrain(values[3], 0, 180);
  int gripper   = constrain(values[4], 0, 180);

  float targetBaseAngle = values[5];

  // =====================
  // MOVE SERVOS (PCA9685)
  // =====================
  pca.setPWM(SHOULDER_CH, 0, angleToPulse(shoulder));
  pca.setPWM(ELBOW_CH, 0, angleToPulse(elbow));
  pca.setPWM(WRIST_CH, 0, angleToPulse(wrist));
  pca.setPWM(WRIST_ROLL_CH, 0, angleToPulse(wristRoll));
  pca.setPWM(GRIPPER_CH, 0, angleToPulse(gripper));

  // =====================
  // MOVE STEPPER
  // =====================
  moveBaseToAngle(targetBaseAngle);

  Serial.println("OK");
}

// =====================
// BASE ANGLE CONTROL
// =====================
void moveBaseToAngle(float targetAngle) {

  float delta = targetAngle - currentBaseAngle;

  // shortest path logic
  if (delta > 180) delta -= 360;
  if (delta < -180) delta += 360;

  int steps = (delta / 360.0) * STEPS_PER_REV;

  moveStepper(steps);

  currentBaseAngle = targetAngle;
}


// =====================
// STEPPER DRIVER
// =====================
void moveStepper(int steps) {

  if (steps == 0) return;

  digitalWrite(DIR_PIN, steps > 0 ? HIGH : LOW);
  steps = abs(steps);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(2000);
  }
}