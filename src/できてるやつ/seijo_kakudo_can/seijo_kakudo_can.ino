#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"
#include <FastLED.h>

#define sin120 0.86602540378

#define NUM_LEDS 5
#define DATA_PIN 21
CRGB leds[NUM_LEDS];

Pid pid0;
Pid pid1;
Pid pid2;
Pid pidpid;

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t state_msg;
static CAN_message_t rxmsg;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

long vel[2];
int flag = 0;
float vx, vy, vt;
float v = 1.5;
int goal = 0;
int goaltg = 0;
int u[4] = {};
int now_dir = 0;//0=北, 1=南, 2=西, 3=東
int pos_dir[4] = {0, 180, 270, 90};
int goal_dir = 0;

void setup() {
  CANTransmitter.begin();
  pid0.init(5.1, 0.06, 0);
  pid1.init(3.8, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(150.0, 0, 1);
  Serial.begin(115200);
  msg.id = 0x200;
  msg.len = 8;
  state_msg.id = 0x0;
  state_msg.len = 2;
  state_msg.buf[0] = 0; //待機中
  state_msg.buf[1] = 0; //北向き
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(255, 255, 0);//黄
  attachInterrupt(2, robo_reset, CHANGE);//reset 0で押下
  attachInterrupt(8, robo_start, CHANGE);//start 0で押下
  pinMode(13, INPUT); //emergency 1で押下
  pinMode(5, OUTPUT); //ブザー
  digitalWrite(5, LOW);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(5, HIGH);
    while (1);
  }
}

void loop() {
  flag = 1;
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int gyro_x = euler.x();
  pidpid.now_value(gyro_x);
  goaltg = pos_dir[goal_dir];

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

  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }

  for (int i = 0; i < 4; i++)u[i] = 0;
  delay(5);
}

void timerInt() {
  if (digitalRead(13) == HIGH) {
    state_msg.buf[0] = 4;
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB(0, 255, 0);//赤
  }
  if (flag) {
    CANTransmitter.write(msg);
    CANTransmitter.write(state_msg);
  }
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
  FastLED.show();
}

void robo_reset() {
  state_msg.buf[0] = 0;
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB(255, 255, 0);//黄
}

void robo_start() {
  if (state_msg.buf[0] == 0)
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB(255, 0, 0);//緑
  state_msg.buf[0] = 1;
}
