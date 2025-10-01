#include <Arduino.h>
#include <HardwareSerial.h>
#include <RS485Master.h>

#include <pinmap.h>
#include <variables.h>
#include <controlling.h>

void setup() {
  Serial.begin(115200);
  leftHip.begin(57600);
  leftKnee.begin(57600);
  leftFoot.begin(57600);

  leftHip.setSpeed(50);
  leftKnee.setSpeed(50);
  leftFoot.setSpeed(50);
  verticalDrive.setSpeed(50);

  leftHip.setAcceleration(1000);
  leftKnee.setAcceleration(1000);
  leftFoot.setAcceleration(300);
  verticalDrive.setAcceleration(300);

  leftHip.setDeceleration(2000);
  leftKnee.setDeceleration(2000);
  leftFoot.setDeceleration(2500);
  verticalDrive.setDeceleration(2000);

}
//  send 5 ms read 5 ms getPos 5 ms

void loop() {
  mainControl();
}