// ============================================================
//                 INCLUDES & HARDWARE SETUP
// ============================================================
#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// ---------------- PIN DEFINITIONS ----------------
const int LEFT_REFLECTIVE  = A0;
const int RIGHT_REFLECTIVE = A2;
const int LDR_PIN          = 2;

// ---------------- MOTOR SHIELD ----------------
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor* leftMotor  = AFMS.getMotor(1);
Adafruit_DCMotor* rightMotor = AFMS.getMotor(2);

// ---------------- BASE SPEEDS ----------------
// DO NOT CHANGE these values (robot tuned for this speed)
int LbaseSpeed = 90;
int RbaseSpeed = 90;

// ============================================================
//                           PID CONTROL
// ============================================================

double kP = 0.3;      // proportional gain
double kI = 0.00;     // integral gain
double kD = 0.01;     // derivative gain

int leftVal = 0;
int rightVal = 0;

int leftErr = 0;
int rightErr = 0;
int lineErr = 0;

int prevLineErr = 0;
double lineIntegral = 0;

// ============================================================
//                SENSOR STATES AND CALIBRATION
// ============================================================

// 0 = both white
// 1 = left black
// 2 = both black
// 3 = right black
int currentState = 0;

const int BLACK_TH = 300;  // common threshold for both sensors

int leftTargVal  = 941;    // calibrated values
int rightTargVal = 743;

// ============================================================
//                    CROSS DETECTION SYSTEM
// ============================================================
int crossCount = 0;
bool inCross = false;
bool counted = false;
unsigned long crossTime = 0;
const int CROSS_TIME = 50; // minimum black duration to confirm cross

// ============================================================
//                        CHEESE FLAGS
// ============================================================
bool push1 = false;
bool push2 = false;
bool push3 = false;
bool push4 = false;
bool push5 = false;

bool CheeseLeft = false;
bool turnDone   = false;

// ============================================================
//                           TIMERS
// ============================================================
unsigned long pastSampleTime = 0;
unsigned long followTime     = 0;
unsigned long startTime      = 0;

bool started = false;

// ============================================================
//                             SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  AFMS.begin();
  pinMode(LDR_PIN, INPUT_PULLUP);

  Serial.println("Waiting for LDR...");

  // Allow calibration phase (black/white check)
  while (digitalRead(LDR_PIN) == LOW) {
    leftVal  = analogRead(LEFT_REFLECTIVE);
    rightVal = analogRead(RIGHT_REFLECTIVE);

    Serial.print(" L= "); Serial.print(leftVal);
    Serial.print(" R= "); Serial.println(rightVal);

    delay(250);
    updateSensorState();
    Serial.println(currentState);
  }

  Serial.println("Starting!");
  pastSampleTime = millis();
}

// ============================================================
//                               LOOP
// ============================================================
void loop() {

  if (!started) {
    startTime = millis();
    started = true;
  }

  // -------- Read sensors --------
  leftVal  = analogRead(LEFT_REFLECTIVE);
  rightVal = analogRead(RIGHT_REFLECTIVE);

  // -------- Update detection systems --------
  CrossDetection();
  updateSensorState();

  // ============================================================
  //                     MAIN STATE MACHINE
  // ============================================================
  switch (crossCount) {

    // ------------------------------------------------------------
    case 0: // Move to first cross
      goForward(0); //process if cross = 0
      break;

    // ------------------------------------------------------------
    case 1: // First and second cheese
      if (!push1) {
        Serial.println("1st Cheese");
        Cheese1(); //process if cross = 1 & !push1
      }

      // Return to line
      followLinePID(); //process when cross = 1 & push1

      // Wait until robot stabilizes on line before Cheese2
      if (push1 && !push2) {
        if (currentState == 3 && (millis() - followTime >= 750)) { 
          Serial.println("2nd Cheese");
          Cheese2(); //process if right detecs black after being stable above the black line (0,75 s)
          break;
        }
      }

      followLinePID();

      if (push1 && push2 && millis() - followTime >= 500)
        counted = false; // allow second cross detection

      break;

    // ------------------------------------------------------------
    case 2: // Third cheese
      if (!push3) {
        Serial.println("3rd Cheese");
        Cheese3();
      }

      followLinePID();

      if (push3 && currentState == 0)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 3: // Drop cheeses at start zone
      if (!CheeseLeft)
        LeaveCheese();

      followLinePID();

      if (currentState == 0)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 4: // Fourth cheese
      if (!turnDone) {
        turnLeft(700);
        followTime = millis();
        turnDone = true;
      }

      if (!push4 && currentState == 3 && (millis() - followTime >= 750)) {
        Serial.println("4th Cheese");
        Cheese4();
      }

      followLinePID();

      if (push4)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 5: // Move cheese 4 to start zone
      if (turnDone) {
        turnRight(500);
        turnDone = false;
      }

      followLinePID();

      if (currentState == 0)
        counted = false;

      CheeseLeft = false;
      break;

    // ------------------------------------------------------------
    case 6: // Drop cheese 4
      if (!CheeseLeft) {
        LeaveCheese();
        goForward(150);
      }

      followLinePID();

      if (currentState == 0)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 7: // Fifth cheese
      if (!turnDone) {
        turnRight(700);
        turnDone = true;
        followTime = millis();
      }

      if (!push5 && currentState == 3 && (millis() - followTime >= 750)) {
        Serial.println("5th Cheese");
        Cheese5();
      }

      followLinePID();

      if (push5 && currentState == 0)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 8: // Move cheese 5 to start
      if (turnDone) {
        turnLeft(600);
        turnDone = false;
      }

      goForward(150);
      followLinePID();

      if (currentState == 0)
        counted = false;

      CheeseLeft = false;
      break;

    // ------------------------------------------------------------
    case 9: // Drop cheese 5
      if (!CheeseLeft) {
        LeaveCheese();
        goForward(150);
      }

      followLinePID();

      if (currentState == 0)
        counted = false;

      break;

    // ------------------------------------------------------------
    case 10:
    case 11: // Continue following line (endgame path)
      if (turnDone) {
        turnLeft(500);
        turnDone = false;
      }

      goForward(150);
      followLinePID();

      if (currentState == 0)
        counted = false;

      CheeseLeft = false;
      break;

    // ------------------------------------------------------------
    case 12: // Final movement → stop
      goForward(150);
      AutoStop();
      break;

    // ------------------------------------------------------------
    default:
      followLinePID();
  }
}

// ============================================================
//                          PID FUNCTION
// ============================================================
void followLinePID() {

  // Basic 2-sensor differential error
  leftErr  = leftTargVal  - leftVal;
  rightErr = rightTargVal - rightVal;
  lineErr  = leftErr - rightErr;

  unsigned long currentTime = millis();
  unsigned long dt_ms = currentTime - pastSampleTime;

  // Small update period
  if (dt_ms >= 5) {

    double dt = dt_ms / 1000.0;
    double dErr = (lineErr - prevLineErr) / dt;

    lineIntegral += lineErr * dt;

    // PID output
    double U = kP * lineErr + kI * lineIntegral + kD * dErr;
    U = constrain(U, -100, 125);

    // Motor speed adjustments
    int leftPWM  = constrain(LbaseSpeed + U, 0, 255);
    int rightPWM = constrain(RbaseSpeed - U, 0, 255);

    leftMotor->setSpeed(leftPWM);
    rightMotor->setSpeed(rightPWM);

    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);

    prevLineErr = lineErr;
    pastSampleTime = currentTime;
  }
}

// ============================================================
//                     CROSS DETECTION
// ============================================================
void CrossDetection() {

  bool bothBlack = (leftVal < BLACK_TH && rightVal < BLACK_TH);

  // Entering a new black region
  if (bothBlack && !inCross) {
    crossTime = millis();
    inCross = true;
  }

  // Cross confirmed
  if (inCross && !counted && millis() - crossTime >= CROSS_TIME) {
    crossCount++;
    counted = true;
    Serial.print("Cross detected: ");
    Serial.println(crossCount);
  }

  // Leaving the black region
  if (!bothBlack)
    inCross = false;
}

// ============================================================
//                  SENSOR STATE CLASSIFICATION
// ============================================================
void updateSensorState() {
  bool leftBlack  = leftVal  < BLACK_TH;
  bool rightBlack = rightVal < BLACK_TH;

  bool leftWhite  = leftVal  > BLACK_TH;
  bool rightWhite = rightVal > BLACK_TH;

  if (leftWhite && rightWhite)      currentState = 0;
  else if (leftBlack && rightBlack) currentState = 2;
  else if (leftBlack && rightWhite) currentState = 1;
  else if (leftWhite && rightBlack) currentState = 3;
  else                              currentState = 0;
}

// ============================================================
//          AUTO STOP (after a specific amount of time)
// ============================================================
void AutoStop() {
  if (millis() - startTime >= 30000) {
    stopM();
    Serial.println("Auto stop.");
    Serial.print("Crosses: "); Serial.println(crossCount);
    Serial.print("CurrentState: "); Serial.println(currentState);
    while (1);
  }
}

// ============================================================
//                      CHEESE FUNCTIONS
// ============================================================
void Cheese1() {
  turnLeft(450);
  goForward(650);
  turnLeft(1000);
  goBack(350);
  turnLeft(250);

  // Return to line until state = both black
  while (currentState != 2) {
    leftVal  = analogRead(LEFT_REFLECTIVE);
    rightVal = analogRead(RIGHT_REFLECTIVE);
    updateSensorState();
    goForward(0);
  }

  goForward(600);
  turnLeft(350);
  push1 = true;
  followTime = millis();
}

void Cheese2() {
  goForward(350);
  turnRight(1000);
  goForward(750);
  goBack(1500);
  turnRight(400);
  goForward(600);
  turnLeft(250);

  while (currentState != 2) {
    leftVal  = analogRead(LEFT_REFLECTIVE);
    rightVal = analogRead(RIGHT_REFLECTIVE);
    updateSensorState();
    goForward(0);
  }

  goForward(300);
  turnRight(750);
  goForward(250);
  push2 = true;
  followTime = millis();
}

void Cheese3() {
  goForward(350);
  turnLeft(1350);
  goForward(750);
  push3 = true;
}

void Cheese4() {
  goForward(250);
  turnLeft(1350);
  goForward(500);
  push4 = true;
}

void Cheese5() {
  goForward(250);
  turnRight(1350);
  goForward(350);
  push5 = true;
}

void LeaveCheese() {
  goForward(850);
  goBack(700);
  turnLeft(1250);
  goForward(250);
  CheeseLeft = true;
}

// ============================================================
//                    TIMED MOVEMENT FUNCTIONS
// ============================================================
void goForward(unsigned long ms) {
  leftMotor->setSpeed(LbaseSpeed);
  rightMotor->setSpeed(LbaseSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  if (ms > 0) delay(ms);
}

void goBack(unsigned long ms) {
  leftMotor->setSpeed(LbaseSpeed);
  rightMotor->setSpeed(LbaseSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  if (ms > 0) delay(ms);
}

void turnLeft(unsigned long ms) {
  leftMotor->setSpeed(LbaseSpeed);
  rightMotor->setSpeed(LbaseSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  if (ms > 0) delay(ms);
}

void turnRight(unsigned long ms) {
  leftMotor->setSpeed(LbaseSpeed);
  rightMotor->setSpeed(LbaseSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  if (ms > 0) delay(ms);
}

void stopM() {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}
