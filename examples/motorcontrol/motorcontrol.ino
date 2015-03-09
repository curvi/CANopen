// demo: CAN-BUS Shield, receive data
// pin 10-13 used here
#include "MotorCtrl.h"
#include <SPI.h>
// #include <stdio.h>

// clear the serial stream and wait for new input
#define SERIAL_WAIT while(Serial.available()>0)\
  {uint8_t junk=Serial.read();}; \
  while(Serial.available()<=0){};

MotorCtrl motorctrl;

uint32_t position = 0;
uint32_t velocity = 0;


void input_parameters (uint32_t* position, uint32_t* velocity) {
  do {
    Serial.print(F("Please enter the goal position: ")); SERIAL_WAIT;
    *position = Serial.parseInt(); Serial.println(*position);
    Serial.print(F("Please enter the velocity: ")); SERIAL_WAIT;
    *velocity = Serial.parseInt(); Serial.println(*velocity);
    Serial.println(F("accept input with yes")); SERIAL_WAIT;
  } while ('y'!=Serial.read());
  Serial.println(F("parameters logged in"));
}


void setup() {
  Serial.begin(115200);

  motorctrl.setup();

  motorctrl.enableOperation(); // motor active

  delay(100);
}


void loop() {
  do {
    input_parameters(&position,&velocity);
    Serial.print(F("position: ")); Serial.print(position);
    Serial.print(F(" | velocity: ")); Serial.println(velocity);
    motorctrl.newSetpoint(position,velocity);
    Serial.println(F("Activate Setpoint with go")); SERIAL_WAIT;
  } while ('g'!=Serial.read());
  // motorctrl.activateNewSetpoint();
  Serial.println(F("GOGO"));
  delay(50);
}
