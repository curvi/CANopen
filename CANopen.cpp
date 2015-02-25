#include "CANopen.h"


MCP_CAN CANopen::can_bus = MCP_CAN(CS_PIN_DEFAULT);
uint8_t CANopen::can_msg_buffer[8] = {0};
uint8_t CANopen::can_receive_buffer[8] = {0};



void CANopen::setup() {
  Serial.println(F("Can setting up"));

  uint8_t init_bool = 0;
  do {
    init_bool = can_bus.begin(CAN_100KBPS);
    if (init_bool==CAN_OK) {
      Serial.print(F("CAN init ok!!\r\n"));
    } else {
      Serial.print(F("CAN init failed!!\r\n"));
    }
    delay(100);
  } while (init_bool!=CAN_OK);
  // set masks to check all ID bits
  // can_bus.init_Mask(0,0,0x7FF); can_bus.init_Mask(1,0,0x7FF);

  // can_bus.init_Filt(0,0,0x581);
  // can_bus.init_Filt(1,0,0x7FF); can_bus.init_Filt(2,0,0x481);
  // can_bus.init_Filt(2,0,0x381); can_bus.init_Filt(3,0,0x7FF); // off
  // can_bus.init_Filt(4,0,0x7FF); can_bus.init_Filt(5,0,0x7FF);
  delay(10);
  Serial.println(F("Can setup complete"));
  return;
}//setup


uint8_t CANopen::read(\
    uint16_t index, uint8_t subIndex,\
    uint32_t* data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_READ,index,subIndex);
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,4);
  uint8_t* ptr = (uint8_t*)data;
  uint8_t len = 0;
  switch (receiveCanMsg()) {
    // requested readings for 8 or 16 or 32 bit data
    case SDO_RESPONSE_READ_8BIT:
      len = 1; break;
    case SDO_RESPONSE_READ_16BIT:
      len = 2; break;
    case SDO_RESPONSE_READ_32BIT:
      len = 4; break;
    default:
      return FAILURE;
  }
  for (uint8_t i=0; i<4; i++) {
    if (i<len) {
      ptr[i] = can_receive_buffer[4+i]; // fill data bytes
    } else {
      ptr[i] = 0x00; // fill the other bytes with 0
    }
  }
  return SUCCESS;
}

uint8_t CANopen::read8bit(\
    uint16_t index, uint8_t subIndex,\
    uint8_t* data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_READ,index,subIndex);
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,4);
  if (receiveCanMsg()==SDO_RESPONSE_READ_8BIT) {
    *data = can_receive_buffer[4]; // first data byte
    return SUCCESS;
  } else {
    return FAILURE; // requested data not received
  }
}

uint8_t CANopen::read16bit(\
    uint16_t index, uint8_t subIndex,\
    uint16_t* data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_READ,index,subIndex);
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,4);
  if (receiveCanMsg()==SDO_RESPONSE_READ_16BIT) {
    uint8_t* ptr = (uint8_t*)data;
    ptr[0] = can_receive_buffer[4];
    ptr[1] = can_receive_buffer[5];
    return SUCCESS;
  } else {
    return FAILURE; // requested data not received
  }
}

uint8_t CANopen::read32bit(\
    uint16_t index, uint8_t subIndex,\
    uint32_t* data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_READ,index,subIndex);
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,4);
  if (receiveCanMsg()==SDO_RESPONSE_READ_32BIT) {
    uint8_t* ptr = (uint8_t*)data;
    ptr[0] = can_receive_buffer[4];
    ptr[1] = can_receive_buffer[5];
    ptr[2] = can_receive_buffer[6];
    ptr[3] = can_receive_buffer[7];
    return SUCCESS;
  } else {
    return FAILURE; // requested data not received
  }
}


uint8_t CANopen::write8bit(\
    uint16_t index, uint8_t subIndex,\
    uint8_t data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_WRITE_8BIT,index,subIndex);
  can_msg_buffer[4] = data;
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,5);
  if (receiveCanMsg()==SDO_RESPONSE_WRITE \
      && can_receive_buffer[1]==(index&0xFF) \
      && can_receive_buffer[2]==((index&0xFF00)>>8) \
      && can_receive_buffer[3]==subIndex) {
    return SUCCESS;
  } else {
    return FAILURE; // writing not terminated by other side
  }
}

uint8_t CANopen::write16bit(\
    uint16_t index, uint8_t subIndex,\
    uint16_t data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_WRITE_16BIT,index,subIndex);
  uint8_t* ptr = (uint8_t*)&data;
  can_msg_buffer[4] = ptr[0];
  can_msg_buffer[5] = ptr[1];
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,6);
  if (receiveCanMsg()==SDO_RESPONSE_WRITE \
      && can_receive_buffer[1]==(index&0xFF) \
      && can_receive_buffer[2]==((index&0xFF00)>>8) \
      && can_receive_buffer[3]==subIndex) {
    return SUCCESS;
  } else {
    return FAILURE; // writing not terminated by other side
  }
}

uint8_t CANopen::write32bit(\
    uint16_t index, uint8_t subIndex,\
    uint32_t data, uint16_t id/*=DEFAULT_NODE_ID*/) {
  composeMsg(SDO_REQUEST_WRITE_32BIT,index,subIndex);
  uint8_t* ptr = (uint8_t*)&data;
  can_msg_buffer[4] = ptr[0];
  can_msg_buffer[5] = ptr[1];
  can_msg_buffer[6] = ptr[2];
  can_msg_buffer[7] = ptr[3];
  sendCanBuffer(id+SDO_COMMAND_ID_BASE,8);
  if (receiveCanMsg()==SDO_RESPONSE_WRITE \
      && can_receive_buffer[1]==(index&0xFF) \
      && can_receive_buffer[2]==((index&0xFF00)>>8) \
      && can_receive_buffer[3]==subIndex) {
    return SUCCESS;
  } else {
    return FAILURE; // writing not terminated by other side
  }
}


uint8_t CANopen::sendCanBuffer(\
    uint16_t id, uint8_t length) {
  return can_bus.sendMsgBuf(id,EXT,length,can_msg_buffer);
}


uint8_t CANopen::composeMsg(\
    uint8_t type_byte, uint16_t index, uint8_t subIndex) {
  can_msg_buffer[0] = type_byte;
  uint8_t *ptr = (uint8_t*)&index;
  can_msg_buffer[1] = ptr[0];
  can_msg_buffer[2] = ptr[1];
  can_msg_buffer[3] = subIndex;
  return SUCCESS;
}


uint8_t CANopen::receiveCanMsg() {
  // wait for message
  uint32_t startTime = millis();
  while(can_bus.checkReceive()!=CANBUS_NEW_MSG){
    if ((millis()-startTime)>CAN_RECEIVE_TIMEOUT_MS) {
      return FAILURE; // timed out
    }
  }
  uint8_t length;
  uint16_t id = can_bus.getCanId();
  while (can_bus.checkReceive()==CANBUS_NEW_MSG) {
    can_bus.readMsgBuf(&length,can_receive_buffer);
#ifdef DEBUG // read out all received messages
    Serial.print(F("Id: "));Serial.println(id,HEX);
    Serial.print(F("data: ")); for(int i = 0; i<length; i++) {
      Serial.print(can_receive_buffer[i],HEX); Serial.print(" "); }
#endif

    // check the type bit, which kind of response it is
    switch (can_receive_buffer[0]) {
      // requested readings for 8,16,32 bit data
      case SDO_RESPONSE_READ_8BIT:
        return SDO_RESPONSE_READ_8BIT;
      case SDO_RESPONSE_READ_16BIT:
        return SDO_RESPONSE_READ_16BIT;
      case SDO_RESPONSE_READ_32BIT:
        return SDO_RESPONSE_READ_32BIT;
      // confirmation of successful write
      case SDO_RESPONSE_WRITE:
        return SDO_RESPONSE_WRITE;
      case SDO_ERROR_CODE: // fall through error
      case SDO_ERROR_CODE+DEFAULT_NODE_ID:
        Serial.println(F("\nERROR"));
        Serial.print(F("Error Message is: "));
        for (uint8_t i=0; i<length; i++) {
          Serial.print(can_receive_buffer[i],HEX);
        }
        break;
      case 0:
        // TODO if PDO message: ACT!,
        Serial.println(F("\nPDO Msg received"));
        Serial.print(F("Message is: "));
        for (uint8_t i=0; i<length; i++) {
          Serial.print(can_receive_buffer[i],HEX);
        }
        break; // maybe another msg is waiting
      default:
        break;
    } // switch type_bit
  } // while msg received
}


uint8_t CANopen::startOperational(uint8_t id/*=DEFAULT_NODE_ID*/) {
  //CAN open message to change from pre-operational state to operational
  can_msg_buffer[0] = 0x01;
  can_msg_buffer[1] = id;
  sendCanBuffer(0x0000,2);
}


uint8_t CANopen::resetNode(uint8_t id/*=DEFAULT_NODE_ID*/) {
  //CAN open message to reset the App on the node
  can_msg_buffer[0] = 0x81;
  can_msg_buffer[1] = id;
  sendCanBuffer(0x0000,2);
}


uint8_t CANopen::sendSyncMsg(uint8_t id/*=DEFAULT_NODE_ID*/) {
  //CAN sync message, after which CANopen devices will respond synchronous,CAN busload can be controlled by Master, in this case its Arduino.
  can_msg_buffer[0] = 0x00;
  can_msg_buffer[1] = id;
  sendCanBuffer(0x0000,2);
}


uint8_t CANopen::readCanBus() {
  // wait for message
  uint32_t startTime = millis();
  while(can_bus.checkReceive()!=CANBUS_NEW_MSG){
    if ((millis()-startTime)>CAN_RECEIVE_TIMEOUT_MS) {
      return FAILURE; // timed out
    }
  }
  uint8_t length;
  uint16_t id = can_bus.getCanId();
  while (can_bus.checkReceive()==CANBUS_NEW_MSG) {
    can_bus.readMsgBuf(&length,can_receive_buffer);
    Serial.print(F("Id: "));Serial.println(id,HEX);
    Serial.print(F("data: ")); for(int i = 0; i<length; i++) {
      Serial.print(can_receive_buffer[i],HEX); Serial.print(" "); }
  }
  return SUCCESS;
}
