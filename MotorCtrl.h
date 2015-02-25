#ifndef MOTORCTRL_H
#define MOTORCTRL_H


#include <Arduino.h>
// make sure DEBUG_MODE = 0 in mcp_can_dfs.h line 35
#include "CANopen.h"


#define DEFAULT_NODE_ID 0x0001
#define CAN_RECEIVE_TIMEOUT_MS 100

#define SUCCESS 1
#define FAILURE 0


// Service Data Object (SDO)
// ------------------------------------
#define SDO_COMMAND_ID_BASE 0x600
#define SDO_REPLY_ID_BASE 0x580

// DATA-Types
//       8Bit  16Bit 32Bit
// write 0x2F  0x2B  0x23
// read  0x4F  0x4B  0x43
#define SDO_REQUEST_READ 0x40
#define SDO_REQUEST_WRITE_8BIT 0x2F
#define SDO_REQUEST_WRITE_16BIT 0x2B
#define SDO_REQUEST_WRITE_32BIT 0x23

#define SDO_RESPONSE_WRITE 0x60
#define SDO_RESPONSE_READ_8BIT 0x4F
#define SDO_RESPONSE_READ_16BIT 0x4B
#define SDO_RESPONSE_READ_32BIT 0x43

#define SDO_ERROR_CODE 0x80


class MotorCtrl
{
public:
  MotorCtrl() {}
  void setup();
  void startController();
  uint8_t applySettings();
  void runPositionMode();

  static CANopen can;

private:
  // static MCP_CAN can_bus;
  // static uint8_t can_msg_buffer[8];
  // static uint8_t can_receive_buffer[8];

};


#endif /* end of include guard: MOTORCTRL_H */
