#pragma once

#define PIN_RX          PE0
#define PIN_TX          PE1

HardwareSerial newSerial(PIN_RX, PIN_TX);