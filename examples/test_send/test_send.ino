// demo: CAN-BUS Shield, send data
// pin 10-13 used here
#include <mcp_can.h>
#include <SPI.h>

#define SENDER_ID 4
#define EXT 0
#define DATA_LENGTH 3

MCP_CAN can_bus(10); // CS pin 9/10 solder bridge
uint8_t canTxValue = 0;

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
}

//Some sample CAN messages
uint8_t msg1[8] = {0, 1, 2, 3, 4, 5, 6, 7};
uint8_t msg2[8] = {0xFF, 0x01, 0x10, 0x0A, 0x00, 0x00, 0x00, 0x00};
uint8_t msg3[4] = {0xFF, 0x01, 0x10, 0x0A};


void loop() {
  canTxValue = serial_input();
  if (canTxValue) {
    Serial.print("canTxValue: "); Serial.println(canTxValue);
    //Create data packet for CAN message
    uint8_t canMsg[8] = {canTxValue,0,0,0,0,0,0,0};
    // send data: id, standard frame, data length, stmp: data buf
    can_bus.sendMsgBuf(SENDER_ID, EXT, DATA_LENGTH, canMsg);
  }
  delay(100);
}

uint16_t serial_input() {
  int16_t num = 0;
  while(Serial.available()) {
    delay(2);
    uint8_t ser = Serial.read();
    if(isDigit(ser)) {
      num = (num*10)+(ser-48);
    }else{ break; }
  }
  if (num) {Serial.println(num);}
  return num;
}
