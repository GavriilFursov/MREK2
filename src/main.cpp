#include <Arduino.h>
#include <HardwareSerial.h>

#include <pinmap.h>
#include <variables.h>
#include <controlling.h>

void setup() {
  Serial.begin(115200);
  newSerial.begin(9600);
}

void loop() {

}