#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"
#include "m3508.h"

#define sin120 0.86602540378

m3508 m3508_1;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

FlexCAN CANTransmitter(1000000);
CAN_message_t txmsg;
CAN_message_t rxmsg;

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
int pid0up, pid1up, pid2up;

float my_yaw_org = 0;
int gyro_x;

void setup() {
  CANTransmitter.begin();
  txmsg.id = 0x200;
  txmsg.len = 8;
  pinMode(13, OUTPUT);
  m3508_1.init();
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
  m3508_1.update(pid0up, pid1up, pid2up, gyro_x);

  static int data[18];
  static int dataNumber = 0;
  static unsigned long lastConnectTime = 0;
  if (Serial1.available() > 0) {
    digitalWrite(13, HIGH);
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

  //右スイッチ上
  if (((data[5] & 0x30) >> 4) == 1) {
    my_yaw_org = euler.x();
    //Serial.println(my_yaw_org);
  }

  //右スイッチ下
  if (((data[5] & 0x30) >> 4) == 2) {
    goaltg = gyro_x * 100;
    goal = 0;
  }

  //左スイッチ上
  if (((data[5] & 0xC0) >> 6) == 1) {
  }
  //左スイッチ中
  else if (((data[5] & 0xC0) >> 6) == 3) {//マニュアル操作
    if (flag01 == 1) {
      goaltg = goaltg * 100;
      flag01 = 0;
    }
    flag02 = 0;

    goal = m3508_1.gyro(gyro_x, goaltg, my_yaw_org, 1);
    vt = goal;
    Serial.print("角出力: ");
    Serial.println(goal);
    Serial.print("目標角: ");
    Serial.println(goaltg / 100);
    Serial.print("VT: ");
    Serial.println(vt);

    stage = 0;
    vx = map(testch[3], 364, 1684, -2000, 2000);//目標最大rpmは5000
    vy = map(testch[2], 364, 1684, -2000, 2000);
    goaltg = goaltg + map(testch[0] - 6, 364, 1684, -100, 100);
    if (abs(vt) < 5)vt = 0;

    m3508_1.setspd(vx, vy, vt, 0);
  }
  //左スイッチ下
  else if (((data[5] & 0xC0) >> 6) == 2) {//停止(フリーではない)
    stage = 0;
    m3508_1.setspd(vx, vy, vt, 1);
    goaltg = gyro_x * 100;
    goal = 0;
  }

  m3508_1.pidto();

  //uをCANに
  for (int i = 0; i < txmsg.len; i++) {
    txmsg.buf[i * 2] = m3508_1.u[i] >> 8;
    txmsg.buf[i * 2 + 1] = m3508_1.u[i] & 0xFF;
  }
  delay(5);
}

//CAN Write Read
void timerInt() {
  if (flag) {
    CANTransmitter.write(txmsg);
  }
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x201) pid0up = rxmsg.buf[2] * 256 + rxmsg.buf[3];
    if (rxmsg.id == 0x202) pid1up = rxmsg.buf[2] * 256 + rxmsg.buf[3];
    if (rxmsg.id == 0x203) pid2up = rxmsg.buf[2] * 256 + rxmsg.buf[3];
    m3508_1.update(pid0up, pid1up, pid2up, gyro_x);
  }
}
