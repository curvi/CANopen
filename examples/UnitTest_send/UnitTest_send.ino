#include "CANopen.h"
#include <SPI.h> // SPI interface with CAN shield

#define DEBUG
#define DT 5000
#define SEND(var) Serial.print(var); Serial.print(F(" "));

CANopen canOpen;
uint32_t starttime = 0;
uint8_t data[8] = {0};
uint8_t reading8 = 0;
uint16_t reading16 = 0;
uint32_t reading = 0;


void setup() {
  Serial.begin(115200);
  canOpen.setup();
  starttime = millis();
}


void loop() {
  uint32_t currenttime = millis();
  if (currenttime-starttime > DT)
  {
    starttime = currenttime;

    if (canOpen.write8bit(0x1401,0x02,0xEF)) {
      Serial.println(F("8 Bit Written successfully!"));
    }
    if (canOpen.write16bit(0x6040,0x00,0x03E8)) {
      Serial.println(F("16 Bit Written successfully!"));
    }
    if (canOpen.write32bit(0x6093,0x01,0x12345678)) {
      Serial.println(F("32 Bit Written successfully!"));
    }
    // read 8 Bit
    canOpen.read(0x6061,0x00,&reading8);
    Serial.print(F("result: "));
    Serial.println(reading8,HEX);
    // read 16 Bit
    canOpen.read(0x6041,0x00,&reading16);
    Serial.print(F("result: "));
    Serial.println(reading16,HEX);
    // read 32 Bit
    canOpen.read(0x6093,0x01,&reading);
    Serial.print(F("result: "));
    Serial.println(reading,HEX);

  }
}
