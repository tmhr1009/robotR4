#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"
#include "m3508.h"
#include "rcan.h"

#define sin120 0.86602540378

m3508 m3508_1;
Pid pidpid;
rcan rcan_1;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

int stage = 0;
int tim = 0;
int flag = 0;
float vx, vy, vt;
int vgyro;
float v = 1.5;
int u[4] = {};
int goal = 0;
int goaltg = 0;
int sita = 0;
int flag01 = 0;
int flag02 = 0;
int movex, movey;

float my_yaw_org = 0;
int gyro_x;

void setup() {
  rcan_1.init();
  pinMode(13, OUTPUT);
  m3508_1.init();
  pidpid.init(150.0, 0, 1);
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

static unsigned long testch[6];

void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro_x = euler.x() - my_yaw_org;
  pidpid.now_value(gyro_x);
  //    Serial.print(euler.x());
  //    Serial.print(",");
  //    Serial.print(my_yaw_org);
  //    Serial.println(",");

  static int data[18];                      //入力データが入る？
  static int dataNumber = 0;                //入力データの数(Serial1.available()の返値)
  static unsigned long lastConnectTime = 0; //直前の通信の時間?
  if (Serial1.available() > 0) {
    for (int dataNum = Serial1.available(); dataNum > 0; dataNum--) {
      if (dataNumber < 0) {
        Serial1.read();
        dataNumber++;
        continue;
      }
      data[dataNumber % 18] = Serial1.read();
      dataNumber++;
      if (dataNumber > 18) {
        dataNumber = 0;
      }
      else if (dataNumber == 18) {
        testch[0] = (((data[1] & 0x07) << 8) | data[0]);          //ch0(364～1024～1684)
        testch[1] = (((data[2] & 0x3F) << 5) | (data[1] >> 3));   //ch1(364～1024～1684)
        testch[2] = (((data[4] & 0x01) << 10) | (data[3] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
        testch[3] = (((data[5] & 0x0F) << 7) | (data[4] >> 1));   //ch3(364～1024～1684)
        if (!(364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684)) {
          for (int i = 1; i < 18; i++) {
            testch[0] = (((data[(1 + i) % 18] & 0x07) << 8) | data[(0 + i) % 18]);  //ch0(364～1024～1684)
            testch[1] = (((data[(2 + i) % 18] & 0x3F) << 5) | (data[(1 + i) % 18] >> 3)); //ch1(364～1024～1684)
            testch[2] = (((data[(4 + i) % 18] & 0x01) << 10) | (data[(3 + i) % 18] << 2) | (data[2] >> 6)); //ch2(364～1024～1684)
            testch[3] = (((data[(5 + i) % 18] & 0x0F) << 7) | (data[(4 + i) % 18] >> 1)); //ch3(364～1024～1684)
            if (364 <= testch[0] && testch[0] <= 1684 && 364 <= testch[1] && testch[1] <= 1684 && 364 <= testch[2] && testch[2] <= 1684 && 364 <= testch[3] && testch[3] <= 1684) {
              dataNumber = -i;
              break;
            }
          }
          if (dataNumber > 18) {
            dataNumber = -1;
          }
        }
        else {
          dataNumber = 0;
        }
      }
    }
    digitalWrite(13, !digitalRead(13)); //プロポ受信したらLEDチカチカ
    flag = 1;
  }
  else {
    flag = 0;
  }

  for (int i = 0; i < 4; i++) {
    u[i] = 0;
  }

  if (((data[5] & 0x30) >> 4) == 1) {
    my_yaw_org = euler.x();
    //Serial.println(my_yaw_org);
  }

  if (((data[5] & 0xC0) >> 6) == 1) {

  }
  else if (((data[5] & 0xC0) >> 6) == 3) {//マニュアル操作
    if (flag01 == 1) {
      goaltg = goaltg * 100;
      flag01 = 0;
    }
    flag02 = 0;

    float my2_yaw_org = my_yaw_org * 100;
    if (goaltg - my2_yaw_org > 36000) goaltg = goaltg - my2_yaw_org - 36000;
    if (goaltg - my2_yaw_org < 0) goaltg = goaltg - my2_yaw_org + 36000;
    if (goaltg - my2_yaw_org / 100 - gyro_x > 180) {
      goal = pidpid.pid_out(goaltg - my2_yaw_org / 100 - 360);
    }
    else if (goaltg / 100 - gyro_x < -180) {
      goal = pidpid.pid_out(goaltg - my2_yaw_org / 100 + 360);
    }
    else {
      goal = pidpid.pid_out(goaltg - my2_yaw_org / 100);
    }
    pidpid.debug();
    vt = goal / 100;

    //    Serial.print("角出力: ");
    //    Serial.println(goal);
    //    Serial.print("目標角: ");
    //    Serial.println(goaltg / 100);

    stage = 0;
    vx = map(testch[3], 364, 1684, -2000, 2000);//目標最大rpmは5000
    vy = map(testch[2], 364, 1684, -2000, 2000);
    goaltg = goaltg + map(testch[0] - 6, 364, 1684, -100, 100);
    if (abs(vt) < 5)vt = 0;

    m3508_1.setspd(vx, vy, vt, 0);
  }
  else if (((data[5] & 0xC0) >> 6) == 2) {//停止(フリーではない)
    stage = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    goaltg = gyro_x * 100;
    goal = 0;
    Serial.print("gyro_x : ");
    Serial.println(gyro_x);
    Serial.print("yaw_org : ");
    Serial.println(my_yaw_org);
    m3508_1.setspd(vx, vy, vt, 1);
  }

  delay(5);
}

void timerInt() {
  if (flag) {
    //    CANTransmitter.write(rcan.msg);
    rcan_1.write();
  }
  rcan_1.read();
//  while ( CANTransmitter.read(rcan.rxmsg) ) {
//    m3508_1.update();
//  }
}
