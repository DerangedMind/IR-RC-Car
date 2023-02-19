#include <Arduino.h>

#define IR_RECEIVE_PIN 10  // connected to pin Y on IR Receiver
#define DECODE_NEC
#define PROCESS_IR_RESULT_IN_MAIN_LOOP
#if defined(PROCESS_IR_RESULT_IN_MAIN_LOOP) || defined(ARDUINO_ARCH_MBED) || defined(ESP32)
volatile bool sIRDataJustReceived = false;
#endif
void ReceiveCompleteCallbackHandler();

#include <IRremote.hpp>

// Right Motor pins
const int rEnable = 9;    // connected to pin 1 ([1,2]EN) on H-bridge
const int rControlB = 3;  // connected to pin 2 (1A) on H-bridge
const int rControlA = 2;  // connected to pin 7 (2A) on H-bridge
int rightMotorEnabled = 0;

// Left Motor pins
const int lEnable = 6;    // connected to pin 9 ([3,4]EN) on H-bridge
const int lControlA = 7;  // connected to pin 10 (3A) on H-bridge
const int lControlB = 8;  // connected to pin 15 (4A) on H-bridge
int leftMotorEnabled = 0;

// Speed pot (0-10k), on car
const int speedPin = A0;  // connected to the potentiometer's output
int motorSpeed = 0;       // speed of the motor

// IR hex codes
const uint8_t FULL_FORWARD_CODE = 0x25;
const uint8_t FULL_BACKWARD_CODE = 0x26;
const uint8_t R_FORWARD_CODE = 0x1A;
const uint8_t L_FORWARD_CODE = 0x2A;
const uint8_t R_BACKWARD_CODE = 0x1B;
const uint8_t L_BACKWARD_CODE = 0x2B;
const uint8_t SPIN_LEFT_CODE = 0x35;
const uint8_t SPIN_RIGHT_CODE = 0x36;
uint8_t RECEIVED_CODE = 0;

bool MOTOR_TIMING_ENABLED = false;
const int MOTOR_ENABLE_PADDING = 400;  // in ms
uint32_t TIME_NOW = millis();
uint32_t TIME_START;

void setup() {
  // Serial.begin(115200);

  pinMode(rEnable, OUTPUT);
  pinMode(rControlA, OUTPUT);
  pinMode(rControlB, OUTPUT);

  pinMode(lEnable, OUTPUT);
  pinMode(lControlA, OUTPUT);
  pinMode(lControlB, OUTPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  IrReceiver.registerReceiveCompleteCallback(receiveIR);

  // pull the enable pin LOW to start
  digitalWrite(rEnable, LOW);
  digitalWrite(lEnable, LOW);
}

void setLeftForward() {
  digitalWrite(lControlA, HIGH);
  digitalWrite(lControlB, LOW);
}

void setLeftBackward() {
  digitalWrite(lControlA, LOW);
  digitalWrite(lControlB, HIGH);
}

void setRightForward() {
  digitalWrite(rControlA, HIGH);
  digitalWrite(rControlB, LOW);
}

void setRightBackward() {
  digitalWrite(rControlA, LOW);
  digitalWrite(rControlB, HIGH);
}

void setMotors(uint8_t leftMotor, uint8_t rightMotor) {
  digitalWrite(lEnable, leftMotor);
  leftMotorEnabled = leftMotor;
  digitalWrite(rEnable, rightMotor);
  rightMotorEnabled = rightMotor;
}

void setDirectionControls() {
  // change the direction the motor spins by talking to the control pins
  // on the H-Bridge
  switch (RECEIVED_CODE) {
    case FULL_FORWARD_CODE:
      setLeftForward();
      setRightForward();
      setMotors(HIGH, HIGH);
      break;
    case FULL_BACKWARD_CODE:
      setLeftBackward();
      setRightBackward();
      setMotors(HIGH, HIGH);
      break;
    case R_FORWARD_CODE:
      setRightForward();
      setMotors(LOW, HIGH);
      break;
    case L_FORWARD_CODE:
      setLeftForward();
      setMotors(HIGH, LOW);
      break;
    case R_BACKWARD_CODE:
      setRightBackward();
      setMotors(LOW, HIGH);
      break;
    case L_BACKWARD_CODE:
      setLeftBackward();
      setMotors(HIGH, LOW);
      break;
    case SPIN_LEFT_CODE:
      setLeftBackward();
      setRightForward();
      setMotors(HIGH, HIGH);
      break;
    case SPIN_RIGHT_CODE:
      setLeftForward();
      setRightBackward();
      setMotors(HIGH, HIGH);
      break;
    default:
      break;
  }
}

// Print a short summary of received data
void printIRDebugInfo() {
  Serial.println();
  IrReceiver.printIRResultShort(&Serial);
  IrReceiver.printIRSendUsage(&Serial);

  if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
    Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
    // We have an unknown protocol here, print more info
    IrReceiver.printIRResultRawFormatted(&Serial, true);
  }
}

void receiveIR() {
  IrReceiver.decode();
  IrReceiver.resume();  // Enable receiving of the next value
  sIRDataJustReceived = true;
}

void runIRCommand() {
  Serial.println("Running IR Command");
  printIRDebugInfo();
  setDirectionControls();
  RECEIVED_CODE = IrReceiver.decodedIRData.command;
  TIME_START = millis();
  MOTOR_TIMING_ENABLED = true;
  motorSpeed = analogRead(speedPin) / 4;
  sIRDataJustReceived = false;
}

void powerMotors() {
  TIME_NOW = millis();
  if (TIME_NOW - TIME_START < MOTOR_ENABLE_PADDING) {
    // Serial.println("Powering Motors");
    if (leftMotorEnabled == HIGH) {
      // Serial.println("LEFT motor ON");
      analogWrite(lEnable, motorSpeed);
    }
    if (rightMotorEnabled == HIGH) {
      // Serial.println("RIGHT motor ON");
      analogWrite(rEnable, motorSpeed);
    }
  } else if (TIME_NOW - TIME_START >= MOTOR_ENABLE_PADDING) {
    // Serial.println("Disabling Motors");
    setMotors(LOW, LOW);
    MOTOR_TIMING_ENABLED = false;
  }
}

// if valid code is received,
//    set direction and enable each motor based on code
//    increase active timers by LATENCY, up to MAX_TIMER
// in main loop,
//    if motor is still active, keep it running
//    reduce time left on motor
void loop() {
  if (sIRDataJustReceived) runIRCommand();
  if (MOTOR_TIMING_ENABLED) powerMotors();
}
