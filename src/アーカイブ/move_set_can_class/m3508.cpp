#include "m3508.h"
#include "pid.h"
#include "rcan.h"
#include <Arduino.h>

#define sin120 0.86602540378

Pid pid0;
Pid pid1;
Pid pid2;
rcan rcan;

void m3508::init() {
  u[4] = {};
  vwx = 0;
  vwy = 0;
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
}

void m3508::update() {
  if (rcan.rxmsg.id == 0x201) {
    pid0.now_value(rcan.rxmsg.buf[2] * 256 + rcan.rxmsg.buf[3]);
  }
  if (rcan.rxmsg.id == 0x202) {
    pid1.now_value(rcan.rxmsg.buf[2] * 256 + rcan.rxmsg.buf[3]);
  }
  if (rcan.rxmsg.id == 0x203) {
    pid2.now_value(rcan.rxmsg.buf[2] * 256 + rcan.rxmsg.buf[3]);
  }
}

void m3508::setspd(float vx, float vy, float vt, int flag) {
  u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);

  for (int i = 0; i < 4; i++)u[i] = 500;

  if (flag == 1) {
    for (int i = 0; i < 4; i++)u[i] = 0;
  }

  Serial.print(u[0]);
  Serial.print(",");
  Serial.print(u[1]);
  Serial.print(",");
  Serial.println(u[2]);

  rcan.senddata(u);
}
