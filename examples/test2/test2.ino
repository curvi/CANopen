#include "CANopen.h"
#include <SPI.h> // SPI interface with CAN shield

#define DEBUG
#define DT 5000
#define SEND(var) Serial.print(var); Serial.print(F(" "));

CANopen motor;
uint32_t starttime = 0;
uint8_t data[8] = {0};
uint16_t reading = 7;


void setup() {
  Serial.begin(115200);
  motor.setup();
  starttime = millis();
}


void loop() {
  uint32_t currenttime = millis();
  if (currenttime-starttime > DT)
  {
    starttime = currenttime;

    reading = motor.write(0x6093,0x01,(uint32_t)0x1234567);
    // motor.read(0x6093,0x01,&reading);
    // motor.write(0x1401,0x02,0xEF);
    Serial.print(F("result: "));
    Serial.println(reading,HEX);

  }
}
