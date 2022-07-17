#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>

#define SW 20

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

int status_flag = 0;
int move_status = 0;
int flag = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pinMode(SW, INPUT);
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
//  MsTimer2::set(2, timerInt);
//  MsTimer2::start();
}

void loop() {
  Serial.println(digitalRead(SW));


  if (status_flag == 0) {
    //0 待機
  } else if (status_flag == 1) {
    //0 待機
  } else if (status_flag == 2) {
    //0 待機
  }

  //非常停止が押された
  if (status_flag == 0) {
    status_flag = 2;
  }

//  msg.id = 0x100;
//  msg.len = 8;
//  for (int i = 0; i < msg.len; i++) {
//    msg.buf[i * 2] = status_flag >> 8;
//    msg.buf[i * 2 + 1] = status_flag & 0xFF;
//  }

  delay(5);
}


//
//void timerInt() {
//  if (flag) {
//    CANTransmitter.write(msg);
//  }
//  while ( CANTransmitter.read(rxmsg) ) {
//  }
//}
