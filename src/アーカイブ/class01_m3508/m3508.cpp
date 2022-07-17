#include "m3508.h"
#include "pid.h"
#include "rot_v.h"
#include <Arduino.h>
#include <FlexCAN.h>

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

void m3508::init() {
  u[4] = {};
  vwx = 0;
  vwy = 0;
}

void m3508::update() {
  if (rxmsg.id == 0x201) {
    pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
  }
  if (rxmsg.id == 0x202) {
    pid1.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
  }
  if (rxmsg.id == 0x203) {
    pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
  }
}

void m3508::setspd(float vx, float vy, float vt) {
  vwx = rot_v.setRx(vx, vy, vt);
  vwy = rot_v.setRy(vx, vy, vt);
  
  u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  for (int i = 0; i < 4; i++)u[i] = 0;
}
