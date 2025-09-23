#include <Arduino.h>
#include <HardwareSerial.h>
#include <RS485Master.h>

#include <pinmap.h>
#include <variables.h>
#include <controlling.h>

void setup() {
  Serial.begin(115200);
  leftHip.begin();
  leftKnee.begin();
  leftHip.setSpeed(50);
  leftKnee.setSpeed(50);
  leftHip.setAcceleration(50);
  leftKnee.setAcceleration(120);
  leftHip.setDeceleration(2000);
  leftKnee.setDeceleration(1500);
}

void loop() {
  mainControl();
}