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
uint32_t reading32 = 0;
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

    // ================================================
    Serial.println(F("\nWriting with explicit size:"));
    if (canOpen.write8bit(0x1401,0x02,0xEF)) {
      Serial.println(F(" 8 bit written successfully!"));
    }
    if (canOpen.write16bit(0x6040,0x00,0x03E8)) {
      Serial.println(F("16 bit written successfully!"));
    }
    if (canOpen.write32bit(0x6093,0x01,0x12345678)) {
      Serial.println(F("32 bit written successfully!"));
    }

    // ================================================
    Serial.println(F("\nReading with explicit size:"));
    canOpen.read8bit(0x6061,0x00,&reading8);
    if (reading8==0x12) {
      Serial.print(F(" 8 bit read successfully! "));
    }
    Serial.print(F("read  8 bit: "));
    Serial.println(reading8,HEX);
    canOpen.read16bit(0x6041,0x00,&reading16);
    if (reading16==0x1234) {
      Serial.print(F("16 bit read successfully! "));
    }
    Serial.print(F("read 16 bit: "));
    Serial.println(reading16,HEX);
    canOpen.read32bit(0x6093,0x01,&reading32);
    if (reading32==0x12345678) {
      Serial.print(F("32 bit read successfully! "));
    }
    Serial.print(F("read 32 bit: "));
    Serial.println(reading32,HEX);

    // ================================================
    Serial.println(F("\nReading without explicit size:"));
    // read without specifying length
    canOpen.read(0x6061,0x00,&reading);
    if (reading==0x12) {
      Serial.print(F(" 8 bit read successfully! "));
    }
    Serial.print(F("read  8 bit: "));
    Serial.println(reading,HEX);
    canOpen.read(0x6041,0x00,&reading);
    if (reading==0x1234) {
      Serial.print(F("16 bit read successfully! "));
    }
    Serial.print(F("read 16 bit: "));
    Serial.println(reading,HEX);
    canOpen.read(0x6093,0x01,&reading);
    if (reading==0x12345678) {
      Serial.print(F("32 bit read successfully! "));
    }
    Serial.print(F("read 32 bit: "));
    Serial.println(reading,HEX);
  }
}
