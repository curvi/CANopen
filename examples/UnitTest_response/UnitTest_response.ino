// demo: CAN-BUS Shield, receive data
// pin 10-13 used here
#include "mcp_can.h"
#include <SPI.h>
// #include <stdio.h>

#define EXT 0
#define ARRAY_SIZE(a)                               \
  ((sizeof(a) / sizeof(*(a))) /                     \
  static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

#define READ_OUT_BUF \
  Serial.print(F(" dat: ")); \
  for(int i = 0; i<8; i++) { \
    Serial.print(buf[i],HEX);Serial.print(" "); \
  }\
  Serial.print(F(" "));

MCP_CAN can_bus(10); // CS pin 9/10 solder bridge
uint8_t len = 0;
uint32_t canId = 0;
uint32_t timestamp = 0;
uint8_t buf[8];

uint8_t expectedHead[6] = {0x40, 0x40, 0x40, \
                          0x2F, 0x2B, 0x23 };
uint8_t expectedIdx[3] = {0x61, 0x41, 0x93};
uint8_t responseMsg[6][8] = {
  {0x4F,0x61,0x60,0x00,0x12,0x00,0x00,0x00}, // return 01
  {0x4B,0x41,0x60,0x00,0x34,0x12,0x00,0x00}, // return 1234
  {0x43,0x93,0x60,0x01,0x78,0x56,0x34,0x12}, // return 1..8
  {0x60,0x01,0x14,0x02,0x00,0x00,0x00,0x00}, // write success
  {0x60,0x40,0x60,0x00,0x00,0x00,0x00,0x00}, // write success
  {0x60,0x93,0x60,0x01,0x00,0x00,0x00,0x00}, // write success
};
uint8_t msgLength[6] = { 5, 6, 8, 4, 4 , 4};


void setup() {
  Serial.begin(115200);

  // init can bus, baudrate: 100k
  uint8_t init_bool = 0;
  do {
    init_bool = can_bus.begin(CAN_100KBPS);
    if (init_bool==CAN_OK) {
      Serial.print("CAN init ok!!\r\n");
    } else {
      Serial.print("CAN init failed!!\r\n");
    }
    delay(100);
  } while (init_bool!=CAN_OK);

  // FILTER: receive only from IDs in range [4,9]
  // for (uint8_t i = 0; i < 6; i++) {
    // can_bus.init_Filt(i, EXT, 0x04+i); // there are 6 filter in mcp2515
  // }
}


void loop() {

  // in case both buffers are filled read both
  if (can_bus.checkReceive()==CAN_MSGAVAIL) {
    // read data, len: data length, buf: data buf
    can_bus.readMsgBuf(&len, buf);
    canId = can_bus.getCanId();
    //Print data to the serial monitor
    Serial.print("CAN ID: "); Serial.print(canId,HEX);
    READ_OUT_BUF;
    // Serial.print(" data len: "); Serial.println(len);
    response(canId,buf,len);
    Serial.println(F("sent"));
  }

  delay(50);
}

void response (uint32_t canId, uint8_t * buf, uint8_t len) {
  for (uint8_t i=0; i < ARRAY_SIZE(expectedHead); i++) {
    if (buf[0] == expectedHead[i]) {
      // head is correct
      if (i<3) {
        if (buf[1]!=expectedIdx[i]) {
          continue;
        }
      }
      for (uint8_t j = 0; j < 8; j++) {
        // copy the correct message
        buf[j] = responseMsg[i][j];
      }
      READ_OUT_BUF;
      can_bus.sendMsgBuf(0x581,0,msgLength[i],&responseMsg[i][0]);
    }
  }
}
