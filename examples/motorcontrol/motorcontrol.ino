// demo: CAN-BUS Shield, receive data
// pin 10-13 used here
#include "MotorCtrl.h"
#include <SPI.h>
// #include <stdio.h>

// extended frame size
#define EXT 0

MotorCtrl motorctrl;

// uint8_t len = 0;
// uint32_t canId = 0;
// uint32_t timestamp = 0;
// uint8_t buf[8];

void setup() {
  Serial.begin(115200);

  motorctrl.setup();
  motorctrl.startController();
  motorctrl.applySettings();

  delay(100);
}


void loop() {


  delay(50);
}
