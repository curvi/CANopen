#ifndef CANOPEN_H
#define CANOPEN_H


#include <Arduino.h>
// CAN_BUS_SHIELD lib from SeeedStudio
// make sure DEBUG_MODE = 0 in mcp_can_dfs.h line 35
#include <mcp_can.h>
// CAUTION! needed in host app:
// #include <SPI.h>     // SPI interface with CAN shield


// read out every msg received
#define DEBUG

#define DEFAULT_NODE_ID 0x0001
#define CAN_RECEIVE_TIMEOUT_MS 100

// see seedstudio can library
#define CANBUS_NO_MSG 4
#define CANBUS_NEW_MSG 3
#define EXT 0
#define CS_PIN_DEFAULT 10
#define CS_PIN_SOLDER_BRIDGE 9
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


class CANopen
{
public:
  CANopen() {}
  void setup();

  uint8_t read(\
      uint16_t index, uint8_t subIndex,\
      uint32_t* data, uint16_t id=DEFAULT_NODE_ID);
  uint8_t read8bit(\
      uint16_t index, uint8_t subIndex,\
      uint8_t* data, uint16_t id=DEFAULT_NODE_ID);
  uint8_t read16bit(\
      uint16_t index, uint8_t subIndex,\
      uint16_t* data, uint16_t id=DEFAULT_NODE_ID);
  uint8_t read32bit(\
      uint16_t index, uint8_t subIndex,\
      uint32_t* data, uint16_t id=DEFAULT_NODE_ID);

  uint8_t write8bit(\
      uint16_t index, uint8_t subIndex,\
      uint8_t data, uint16_t id=DEFAULT_NODE_ID);
  uint8_t write16bit(\
      uint16_t index, uint8_t subIndex,\
      uint16_t data, uint16_t id=DEFAULT_NODE_ID);
  uint8_t write32bit(\
      uint16_t index, uint8_t subIndex,\
      uint32_t data, uint16_t id=DEFAULT_NODE_ID);


  uint8_t composeMsg(\
      uint8_t type_byte, uint16_t index,uint8_t subIndex);
  uint8_t receiveCanMsg();
  uint8_t sendCanBuffer(uint16_t id, uint8_t length);

  // special message functions
  uint8_t startOperational(uint8_t id=DEFAULT_NODE_ID);
  uint8_t resetNode(uint8_t id=DEFAULT_NODE_ID);
  uint8_t sendSyncMsg(uint8_t id=DEFAULT_NODE_ID);

  uint8_t readCanBus();

private:
  static MCP_CAN can_bus;
  static uint8_t can_msg_buffer[8];
  static uint8_t can_receive_buffer[8];
};


#endif /* end of include guard: CANOPEN_H */
