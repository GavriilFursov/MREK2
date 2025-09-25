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
  verticalDrive.begin();
  
  // verticalDrive.setPulseAbsolutePosition(-300000 * 5);
  // verticalDrive.setPulseAbsolutePosition(0);
  // verticalDrive.setSpeed(300);
  // verticalDrive.run(1);
  leftHip.setSpeed(50);
  leftKnee.setSpeed(50);
  leftHip.setAcceleration(200);
  leftKnee.setAcceleration(50);
  leftHip.setDeceleration(2700);
  leftKnee.setDeceleration(2500);
}

void loop() {
  mainControl();
}