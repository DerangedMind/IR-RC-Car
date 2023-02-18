#include <Arduino.h>

#define IR_RECEIVE_PIN 10  // connected to pin Y on IR Receiver
#define DECODE_NEC

// Right Motor pins
const int rEnable = 9;    // connected to pin 1 ([1,2]EN) on H-bridge
const int rControlB = 3;  // connected to pin 2 (1A) on H-bridge
const int rControlA = 2;  // connected to pin 7 (2A) on H-bridge

// Left Motor pins
const int lEnable = 6;    // connected to pin 9 ([3,4]EN) on H-bridge
const int lControlA = 7;  // connected to pin 10 (3A) on H-bridge
const int lControlB = 8;  // connected to pin 15 (4A) on H-bridge

// On-Car buttons (mostly for debugging)
const int directionPin = 4;    // connected to the switch for direction
const int powerTogglePin = 5;  // connected to the switch for turning the motor on and off
const int speedPin = A0;       // connected to the potentiometer's output

// // On-car button states, used for debugging
// int motorPowerState = 0;               // current state of the on/off switch
// int prevMotorPowerState = 0;           // previous position of the on/off switch
// int directionSwitchState = 0;          // current state of the direction switch
// int previousDirectionSwitchState = 0;  // previous state of the direction switch

// int motorEnabled = 0;    // Turns the motor on/off
// int motorSpeed = 0;      // speed of the motor
// int motorDirection = 1;  // current direction of the motor

#define PROCESS_IR_RESULT_IN_MAIN_LOOP
#if defined(PROCESS_IR_RESULT_IN_MAIN_LOOP) || defined(ARDUINO_ARCH_MBED) || defined(ESP32)
volatile bool sIRDataJustReceived = false;
#endif

void ReceiveCompleteCallbackHandler();

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

// Right & Left Motor states
int rightMotorState = 0;
int prevRightMotorState = 0;

int leftMotorState = 0;
int prevLeftMotorState = 0;

int rightMotorEnabled = 0;
int leftMotorEnabled = 0;

#include <IRremote.hpp>

void setup() {
  Serial.begin(115200);
  // initialize the inputs and outputs
  pinMode(directionPin, INPUT);
  pinMode(powerTogglePin, INPUT);

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

// void toggleMotorPowerStateDEBUG() {
//   // if the on/off button changed state since the last loop()
//   if (motorPowerState != prevMotorPowerState) {
//     // change the value of motorEnabled if pressed
//     if (motorPowerState == HIGH) {
//       motorEnabled = !motorEnabled;
//     }
//   }
// }

// void toggleDirectionDEBUG() {
//   // if the direction button changed state since the last loop()
//   if (directionSwitchState != previousDirectionSwitchState) {
//     // change the value of motorDirection if pressed
//     if (directionSwitchState == HIGH) {
//       motorDirection = !motorDirection;
//     }
//   }

//   if (motorDirection == 1) {
//     digitalWrite(rControlA, HIGH);
//     digitalWrite(rControlB, LOW);

//     digitalWrite(lControlA, HIGH);
//     digitalWrite(lControlB, LOW);
//   } else {
//     digitalWrite(rControlA, LOW);
//     digitalWrite(rControlB, HIGH);

//     digitalWrite(lControlA, LOW);
//     digitalWrite(lControlB, HIGH);
//   }
// }

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
  digitalWrite(rEnable, rightMotor);
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

// void updateCarBtnStateDEBUG() {
//   motorPowerState = digitalRead(powerTogglePin);
//   delay(1);
//   directionSwitchState = digitalRead(directionPin);
//   motorSpeed = analogRead(speedPin) / 4;  // divide pot value by 4 to get a value that can be used for PWM
// }

// void updateStateDEBUG() {
//   updateCarBtnStateDEBUG();
//   toggleMotorPowerStateDEBUG();
//   toggleDirectionDEBUG();
// }

// void powerMotor() {
//   // if the motor is supposed to be on
//   if (motorEnabled == 1) {
//     // PWM the enable pin to vary the speed
//     analogWrite(rEnable, motorSpeed);
//     analogWrite(lEnable, motorSpeed);
//   } else {  // if the motor is not supposed to be on
//     //turn the motor off
//     analogWrite(rEnable, 0);
//     analogWrite(lEnable, 0);
//   }
// }

// void resolveFrame() {
//   // save the current on/off switch state as the previous
//   previousDirectionSwitchState = directionSwitchState;
//   // save the current switch state as the previous
//   prevMotorPowerState = motorPowerState;
// }

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
  printIRDebugInfo();
  IrReceiver.resume();  // Enable receiving of the next value
  sIRDataJustReceived = true;
}

void runIRCommand() {
  if (IrReceiver.decodedIRData.command == L_FORWARD_CODE) {
    // motorEnabled = !motorEnabled;
    //   analogWrite(rEnable, motorSpeed);
  } else if (IrReceiver.decodedIRData.command == R_FORWARD_CODE) {
    //   analogWrite(rEnable, 0);
  }
  // } else if (IrReceiver.decodedIRData.command == L_BACKWARD_CODE) {
  //   analogWrite(rEnable, motorSpeed);
  // } else if (IrReceiver.decodedIRData.command == R_BACKWARD_CODE) {
  //   analogWrite(rEnable, 0);
  //   analogWrite(lEnable, 0);
  // }
}

void loop() {
  //   updateStateDEBUG();
  if (sIRDataJustReceived) {
    // receiveIR();
  }
  //   receiveIR();
  //   powerMotor();
  //   resolveFrame();
}
