#include "m3508.h"
#include "pid.h"
#include "rotV.h"
#include <Arduino.h>
#include <FlexCAN.h>

#define sin120 0.86602540378

static CAN_message_t msg;
static CAN_message_t rxmsg;

Pid pid0;
Pid pid1;
Pid pid2;
rotV rotV_1;

void m3508::init() {
  u[4] = {};
  vwx = 0;
  vwy = 0;
  rotV_1.init();
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
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

void m3508::setspd(float vx, float vy, float vt, int gyro_x) {
  vwx = rotV_1.setRx(vx, vy, gyro_x);
  vwy = rotV_1.setRy(vx, vy, gyro_x);

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
