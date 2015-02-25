#include "MotorCtrl.h"


CANopen MotorCtrl::can = CANopen();
uint16_t CANopen::controlWord = 0;



void MotorCtrl::setup() {
  Serial.println(F("Motor control setting up"));

  can.setup();

  // set DIN 8 HIGH to activate CAN-BUS
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  do { // wait for bootup msg
    readCanBus();
  } while (canID!=0x700);

  startOperational();

  setOperationMode(MODE_POSITION);

  setMotorReady();

  // enableOperation();

  delay(10);
  Serial.println(F("Motor control set up done"));
  return;
}//setup


void MotorCtrl::startController() {
  can.sendSyncMsg();
  can.readCanBus();
  can.startOperational();
  can.readCanBus();
  return;
}


uint8_t MotorCtrl::applySettings() {
  uint8_t ok[1] = {FAILURE};
  ok[0] = can.write8bit(id,sid,0x0);
  if (ok[0]==SUCCESS) {
    return SUCCESS;
  } else {
    return FAILURE;
  }
}


uint8_t readControlWord() {
  return can.read16bit(0x6040,0x00,&controlWord);
}
uint8_t writeControlWord(uint16_t set) {
  // need to wait? while (controlWord!=..)
  can.write16bit(0x6040,0x00,controlWord|set);
  return readControlWord();
}
uint8_t setMotorReady() {
  return writeControlWord(0x06);
}
uint8_t enableOperation() {
  // set motor on and enable operation
  return writeControlWord(0x0F);
}
void motorHalt() {
  // save way to bring the motor to a halt
  // triggers the target reached flag
  return writeControlWord(0x80);
}
void activateNewSetpoint() {
  return writeControlWord(0x08);
}


uint8_t setOperationMode(uint8_t mode) {
  can.write8bit(0x6060,0x00,mode);
  uint8_t newMode = 255;
  do {
    can.read8bit(0x6061,0x00,&newMode);
  } while (newMode!=mode);
  return SUCCESS;
}
