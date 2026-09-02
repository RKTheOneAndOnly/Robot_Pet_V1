// Standalone servo bring-up test for the ESP32-CAM board - no camera, no
// WiFi, no display, no serial input needed. Both servos continuously sweep
// 0 -> 180 -> 0 so you can just watch them move. If they don't move here,
// it's power/wiring/pin, not a software/timer conflict with the camera.


// Run using & "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e servo_test -t upload

#include <Arduino.h>
#include <ESP32Servo.h>
#include "wifi_config.h"

#define FLASH_LED_PIN 4

Servo panServo;
Servo tiltServo;

const int STEP_DEG = 1;
const int STEP_DELAY_MS = 15; // ~2.7s for a full 0->180 sweep

bool ledState = false;

void heartbeat(int blinkPeriodMs) {
  static unsigned long lastToggle = 0;
  unsigned long now = millis();
  if (now - lastToggle >= (unsigned long)blinkPeriodMs) {
    lastToggle = now;
    ledState = !ledState;
    digitalWrite(FLASH_LED_PIN, ledState);
  }
}

void setup() {
  pinMode(FLASH_LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(300);
  Serial.println("Servo sweep test: 0 -> 180 -> 0, no input needed.");

  for (int i = 0; i < 3; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(200);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(200);
  }

  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(SERVO_PAN_PIN, SERVO_PAN_MIN_PULSE_US, SERVO_PAN_MAX_PULSE_US);
  tiltServo.attach(SERVO_TILT_PIN, SERVO_TILT_MIN_PULSE_US, SERVO_TILT_MAX_PULSE_US);

  panServo.write(SERVO_PAN_MIN_ANGLE);
  tiltServo.write(SERVO_TILT_MIN_ANGLE);
  delay(500);
}

void loop() {
  for (int a = 0; a <= 180; a += STEP_DEG) {
    if (a >= SERVO_PAN_MIN_ANGLE && a <= SERVO_PAN_MAX_ANGLE) panServo.write(a);
    if (a >= SERVO_TILT_MIN_ANGLE && a <= SERVO_TILT_MAX_ANGLE) tiltServo.write(a);
    heartbeat(150);
    delay(STEP_DELAY_MS);
  }
  delay(400);
  for (int a = 180; a >= 0; a -= STEP_DEG) {
    if (a >= SERVO_PAN_MIN_ANGLE && a <= SERVO_PAN_MAX_ANGLE) panServo.write(a);
    if (a >= SERVO_TILT_MIN_ANGLE && a <= SERVO_TILT_MAX_ANGLE) tiltServo.write(a);
    heartbeat(500);
    delay(STEP_DELAY_MS);
  }
  delay(400);
}
