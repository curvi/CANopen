#include "MotorCtrl.h"


CANopen MotorCtrl::can = CANopen();

// MCP_CAN CANopen::can_bus = MCP_CAN(CS_PIN_DEFAULT);
// uint8_t CANopen::can_msg_buffer[8] = {0};
// uint8_t CANopen::can_receive_buffer[8] = {0};



void MotorCtrl::setup() {
  Serial.println(F("Motor control setting up"));

  can.setup();

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


void MotorCtrl::runPositionMode() {
  can.write8bit(id,sid,0x0);
  return;
}
