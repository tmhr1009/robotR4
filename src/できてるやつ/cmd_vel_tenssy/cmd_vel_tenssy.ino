#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"

#define sin120 0.86602540378

Pid pid0;
Pid pid1;
Pid pid2;
Pid pidpid;

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

long vel[2];
int flag = 0;
float vx, vy, vt;
float v = 1.5;
int u[4] = {};

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pid0.init(5.1, 0.06, 0);
  pid1.init(3.8, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(150.0, 0, 1);
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
}

static unsigned long testch[6];

void loop() {
  flag = 1;

  for (int i = 0; i < 4; i++) {
    u[i] = 0;
  }

  Serial.print("vel[] : ");
  Serial.print(vel[0]);
  Serial.print(", ");
  Serial.print(vel[1]);
  Serial.println();



  vx = vel[0] / 1.52;
  vy = vel[1] / 1.52;
  vt = min(max(vt, -250), 250);

  u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);

  msg.id = 0x200;
  msg.len = 8;
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }

  for (int i = 0; i < 4; i++)u[i] = 0;
  delay(5);
}

void timerInt() {
  if (flag) {
    CANTransmitter.write(msg);
  }
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x70) {
      vel[0] = -1 * map(rxmsg.buf[0] + rxmsg.buf[1] * 256, 0, 65535, -1023, 1023);
      vel[1] = -1 * map(rxmsg.buf[2] + rxmsg.buf[3] * 256, 0, 65535, 1023, -1023);
    }
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
}
