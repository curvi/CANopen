// demo: CAN-BUS Shield, receive data
// pin 10-13 used here
#include "mcp_can.h"
#include <SPI.h>
// #include <stdio.h>

#define EXT 0

MCP_CAN can_bus(10); // CS pin 9/10 solder bridge
uint8_t flag_received = 0;
uint8_t len = 0;
uint32_t canId = 0;
uint32_t timestamp = 0;
uint8_t buf[8];

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
  // msg receive interrupt
  attachInterrupt(0, MCP2515_ISR, FALLING);

  // FILTER: receive only from IDs in range [4,9]
  for (uint8_t i = 0; i < 6; i++) {
    // can_bus.init_Filt(i, EXT, 0x04+i); // there are 6 filter in mcp2515
  }
}

void MCP2515_ISR() {
  flag_received = 1;
}

void loop() {
  // check if data was recieved
  if(flag_received) {
    flag_received = 0; // clear flag

    // in case both buffers are filled read both
    while (CAN_MSGAVAIL == can_bus.checkReceive()) {
      can_bus.readMsgBuf(&len, buf); // read data, len: data length, buf: data buf
      canId = can_bus.getCanId();
      //Print data to the serial monitor
      Serial.println("CAN_BUS got data!");
      Serial.print("CAN ID: "); Serial.print(canId);
      Serial.print(" data len = "); Serial.println(len);
      for(int i = 0; i<len; i++) {
        Serial.print(buf[i]);Serial.print(" ");
      }
      Serial.println();
    }
  }


  delay(50);
}
