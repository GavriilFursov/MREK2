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

  leftHip.setAcceleration(100);
  leftKnee.setAcceleration(50);
  leftFoot.setAcceleration(300);

  leftHip.setDeceleration(3200);
  leftKnee.setDeceleration(3000);
  leftFoot.setDeceleration(3000);
}

//  send 5 ms read 5 ms getPos 5 ms

void loop() {
  // mainControl();
  float elapsed_time = (millis() - 0) / 1000.0f;
  float trajectory_progress = fmod(elapsed_time, ORIGINAL_FOURIER_CYCLE_TIME * time_scale_factor);
  float normalized_time = trajectory_progress / (ORIGINAL_FOURIER_CYCLE_TIME * time_scale_factor);
  fourierTrajectoryA0(normalized_time);

  Serial.print(Z);
  Serial.print(" ");
  Serial.println(heightToLength(Z, verticalDriver));
}