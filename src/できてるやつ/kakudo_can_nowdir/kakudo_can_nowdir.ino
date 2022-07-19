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
static CAN_message_t motor_msg;
static CAN_message_t dir_msg;
static CAN_message_t rxmsg;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

long vel[2];
float vx, vy, vt;
float v = 1.5;
int goal = 0;
int goaltg = 0;
int u[4] = {};
int now_dir = 0;//0=北, 1=南, 2=西, 3=東
int pos_dir[4] = {0, 180, 270, 90};
int goal_dir = 0;
int motor_flag = 0;
int ang_flag = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pid0.init(5.1, 0.06, 0);
  pid1.init(3.8, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(150.0, 0, 1);
  Serial.begin(115200);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int gyro_x = euler.x();
  pidpid.now_value(gyro_x);
  goaltg = pos_dir[goal_dir];

  if (pos_dir[now_dir] == pos_dir[goal_dir]) {
    now_dir = goal_dir;
  }

  dir_msg.id = 0x000;
  dir_msg.len = 8;
  dir_msg.buf[1] = now_dir;

  if (goaltg > 360)goaltg = goaltg - 360;
  if (goaltg < 0)goaltg = goaltg + 360;
  if (goaltg - gyro_x > 180) {
    goal = pidpid.pid_out(goaltg - 360);
  }
  else if (goaltg - gyro_x < -180) {
    goal = pidpid.pid_out(goaltg + 360);
  }
  else {
    goal = pidpid.pid_out(goaltg);
  }
  vt = goal;

  //goaltg = goaltg + pos_dir[goal_dir];

  vx = vel[0];
  vy = vel[1];

  Serial.print("vel[] : ");
  Serial.print(vel[0]);
  Serial.print(", ");
  Serial.print(vel[1]);
  Serial.println();
  Serial.print("goaltg : ");
  Serial.print(pos_dir[goal_dir]);
  Serial.println();
  Serial.print("vt : ");
  Serial.print(vt);
  Serial.println();
  Serial.println();

  vt = min(max(vt, -500), 500);

  u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);
  Serial.print("motor u[]: ");
  Serial.print(u[0]);
  Serial.print(", ");
  Serial.print(u[1]);
  Serial.print(", ");
  Serial.print(u[2]);
  Serial.println();



  motor_msg.id = 0x200;
  motor_msg.len = 8;
  for (int i = 0; i < motor_msg.len; i++) {
    motor_msg.buf[i * 2] = u[i] >> 8;
    motor_msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }

  for (int i = 0; i < 4; i++)u[i] = 0;
  delay(5);
}

void timerInt() {
  CANTransmitter.write(motor_msg);
  CANTransmitter.write(dir_msg);

  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x70) {
      vel[0] = -1 * map(rxmsg.buf[0] + rxmsg.buf[1] * 256, 0, 65535, -1023, 1023);
      vel[1] = -1 * map(rxmsg.buf[2] + rxmsg.buf[3] * 256, 0, 65535, 1023, -1023);
    }
    if (rxmsg.id == 0x71) {
      goal_dir = rxmsg.buf[0];
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
