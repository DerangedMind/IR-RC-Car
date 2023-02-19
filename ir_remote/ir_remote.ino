#define DELAY_AFTER_REPEAT 110
#define DELAY_AFTER_SEND 1000  // Delay is required between IR signals -- account for this in the car's code
#define DELAY_AFTER_LOOP 2000
#define IR_SEND_PIN 3
// #define TONE_PIN            4
// #define APPLICATION_PIN     5
/* An IR LED must be connected to Arduino PWM pin 3.*/ /*The <IRremote.h> library is referenced*/
#define DISABLE_CODE_FOR_RECEIVER                      // saves 450 bytes if receiver not used
// #define DO_NOT_USE_FEEDBACK_LED
#include <IRremote.hpp>


// Button pins
const int lForward = 6;
const int rForward = 8;
const int lBackward = 7;
const int rBackward = 9;

// Button on/off states
int lForwardState = 0;
int rForwardState = 0;
int lBackwardState = 0;
int rBackwardState = 0;

// Code recently sent, used for sending repeat code if button is held down
bool messageSending = false;

/*
 * Set up the data to be sent.
 * For most protocols, the data is build up with a constant 8 (or 16 byte) address
 * and a variable 8 bit command.
 * There are exceptions like Sony and Denon, which have 5 bit address.
 */
const uint8_t carAddress = 0xBE;
const uint8_t sRepeats = 3;

const uint8_t FULL_FORWARD_CODE = 0x25;
const uint8_t FULL_BACKWARD_CODE = 0x26;
const uint8_t R_FORWARD_CODE = 0x1A;
const uint8_t L_FORWARD_CODE = 0x2A;
const uint8_t R_BACKWARD_CODE = 0x1B;
const uint8_t L_BACKWARD_CODE = 0x2B;
const uint8_t SPIN_LEFT_CODE = 0x35;
const uint8_t SPIN_RIGHT_CODE = 0x36;

uint8_t CURRENT_BTN_CODE = 0;
uint8_t PREV_BTN_CODE = 0;

void setup() {
  pinMode(lForward, INPUT);
  pinMode(rForward, INPUT);
  pinMode(lBackward, INPUT);
  pinMode(rBackward, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  IrSender.begin();
  delay(1000);
  IrSender.sendNEC(0x00, 0x34, sRepeats);
  delay(DELAY_AFTER_SEND);
}

void updateButtonStates() {
  delay(100);
  lForwardState = digitalRead(lForward);
  rForwardState = digitalRead(rForward);
  lBackwardState = digitalRead(lBackward);
  rBackwardState = digitalRead(rBackward);
}

void updateBtnCode() {
  if (lForwardState == HIGH && rForwardState == HIGH) {
    CURRENT_BTN_CODE = FULL_FORWARD_CODE;
  } else if (lBackwardState == HIGH && rBackwardState == HIGH) {
    CURRENT_BTN_CODE = FULL_BACKWARD_CODE;
  } else if (lForwardState == HIGH && rBackwardState == HIGH) {
    CURRENT_BTN_CODE = SPIN_RIGHT_CODE;
  } else if (lBackwardState == HIGH && rForwardState == HIGH) {
    CURRENT_BTN_CODE = SPIN_LEFT_CODE;
  } else if (lForwardState == HIGH) {
    CURRENT_BTN_CODE = L_FORWARD_CODE;
  } else if (rForwardState == HIGH) {
    CURRENT_BTN_CODE = R_FORWARD_CODE;
  } else if (lBackwardState == HIGH) {
    CURRENT_BTN_CODE = L_BACKWARD_CODE;
  } else if (rBackwardState == HIGH) {
    CURRENT_BTN_CODE = R_BACKWARD_CODE;
  } else {
    CURRENT_BTN_CODE = 0;
  }
}

// are we still holding down the button?
//    if yes, send repeat code 0xFF
//    if no, delay appropriately and reset `messageSending` to false
void repeatOrResolve() {
  if (CURRENT_BTN_CODE == PREV_BTN_CODE) {
    IrSender.sendNECRepeat();
    delay(DELAY_AFTER_REPEAT);
  } else if (CURRENT_BTN_CODE != 0) {

  } else {
    // after a command is released, delay & reset state
    delay(DELAY_AFTER_LOOP);
    messageSending = false;
    PREV_BTN_CODE = 0;
    CURRENT_BTN_CODE = 0;
  }
}

void loop() {
  updateButtonStates();
  updateBtnCode();
  if (CURRENT_BTN_CODE != 0) {
    // check if a command was just sent
    if (messageSending) {
      if (CURRENT_BTN_CODE == PREV_BTN_CODE) {
        repeatOrResolve();
      } else {
        // else if a new button as pressed recently, send that instead
        IrSender.sendNEC(carAddress & 0xFF, CURRENT_BTN_CODE, sRepeats);
        PREV_BTN_CODE = CURRENT_BTN_CODE;
        delay(DELAY_AFTER_REPEAT);
      }
    } else {
      // else, if a new button was pressed after delay, emit IR code!
      IrSender.sendNEC(carAddress & 0xFF, CURRENT_BTN_CODE, sRepeats);
      PREV_BTN_CODE = CURRENT_BTN_CODE;
      messageSending = true;
      delay(DELAY_AFTER_REPEAT);
    }
  }
}
