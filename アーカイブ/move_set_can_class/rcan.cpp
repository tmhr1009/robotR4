#include "m3508.h"
#include "pid.h"
#include "rcan.h"
#include <Arduino.h>
#include <FlexCAN.h>

#define sin120 0.86602540378

m3508 m3508_3;

FlexCAN CANTransmitter(1000000);

void rcan::init() {
  CANTransmitter.begin();
}

void rcan::senddata(int u[4]) {
  msg.id = 0x200;
  msg.len = 8;

  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  for (int i = 0; i < 4; i++) {
    Serial.print("u[");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(u[i]);
  }
  for (int i = 0; i < 4; i++)u[i] = 0;
}

void rcan::write() {
  CANTransmitter.write(msg);
  Serial.println("aaaaaaaaa");
}

void rcan::read() {
  while ( CANTransmitter.read(rxmsg) ) {
    m3508_3.update();
  }
}
