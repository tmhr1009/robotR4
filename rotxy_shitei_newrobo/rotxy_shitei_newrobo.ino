#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"
#include "melody.h"

#define sin120 0.86602540378

Pid pid0;
Pid pid1;
Pid pid2;
Pid pidpid;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

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
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(400.0, 0, 1);
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
  msg.id = 0x200;
  msg.len = 8;
  for (int i = 0; i < 4; i++) {
    u[i] = 0;
  }

  if (((data[5] & 0x30) >> 4) == 1) {
    my_yaw_org = euler.x();

    //    Serial.println(my_yaw_org);
  }

  if (((data[5] & 0xC0) >> 6) == 1) {
    flag01 = 1;
    if (goaltg - my_yaw_org > 900) goaltg = goaltg - my_yaw_org / 100;
    if (goaltg - my_yaw_org > 360) goaltg = goaltg - my_yaw_org - 360;
    if (goaltg - my_yaw_org < 0) goaltg = goaltg - my_yaw_org + 360;
    if (goaltg - my_yaw_org - gyro_x > 180) {
      goal = pidpid.pid_out(goaltg - my_yaw_org - 360);
    }
    else if (goaltg - gyro_x < -180) {
      goal = pidpid.pid_out(goaltg - my_yaw_org + 360);
    }
    else {
      goal = pidpid.pid_out(goaltg - my_yaw_org);
    }
    pidpid.debug();
    vt = goal;

    switch (stage) {
      case 0:
        stage++;
        tim = millis();
        break;
      case 1:
        if (x_str100(tim, millis(), gyro_x) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 2:
        vx = 0;
        vy = 0;
        vt = 0;
        u[0] = 0;
        u[1] = 0;
        u[2] = 0;
        break;
    }
  }
  else if (((data[5] & 0xC0) >> 6) == 3) {//マニュアル操作

    if (((data[5] & 0x30) >> 4) == 2) {//左真ん中 右下
      //移動量指定
      if (flag02 == 0) {
        tim = millis();
        movex = -300;
        movey = 0;
        flag02 = 1;
      }

      int movet01 = 3.333 * movex;
      if (movex < 0) movet01 = movet01 * -1;
      int movet02 = 3.333 * movey;
      if (movey < 0) movet02 = movet02 * -1;

      Serial.print("movex: ");
      Serial.println(movex);
      Serial.print("movet01: ");
      Serial.println(movet01);
      Serial.print("flaggggggg02: ");
      Serial.println(millis() - tim);

      if (millis() - tim < movet01) {
        if (movex < 0) vx = -600;
        else vx = 600;
      } else {
        vx = 0;
      }

      if (millis() - tim < movet02) {
        if (movey < 0) vy = -600;
        else vy = 600;
      } else {
        vy = 0;
      }
    } else { //マニュアル操作
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

      Serial.print("角出力: ");
      Serial.println(goal);
      Serial.print("目標角: ");
      Serial.println(goaltg / 100);

      stage = 0;
      vx = map(testch[3], 364, 1684, -2000, 2000);//目標最大rpmは5000
      vy = map(testch[2], 364, 1684, -2000, 2000);
      //      goaltg = goaltg + map(testch[0] - 6, 364, 1684, -100, 100);
      goaltg = goaltg;
      if (abs(vt) < 5)vt = 0;

      Serial.print("VT: ");
      Serial.println(vt);
      Serial.print("右スティック: ");
      Serial.println(map(testch[0] - 6, 364, 1684, -100, 100));
    }
    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    vwx = vwx / 1.52;
    vwy = vwy / 1.52;
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  }
  else if (((data[5] & 0xC0) >> 6) == 2) {//停止(フリーではない)
    stage = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    goaltg = gyro_x;
    goal = 0;
    Serial.print("gyro_x : ");
    Serial.println(gyro_x);
    Serial.print("yaw_org : ");
    Serial.println(my_yaw_org);
  }

  //  Serial.print(vx);
  //  Serial.print(",");
  //  Serial.print(vy);
  //  Serial.print(",");
  //  Serial.println(vt);
  //  Serial.print("goaltg ,");
  //  Serial.println(goaltg);

  u[0] = min(max(u[0], -2500), 2500);
  u[1] = min(max(u[1], -2500), 2500);
  u[2] = min(max(u[2], -2500), 2500);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);

  for (int i = 0; i < 3; i++) {
    Serial.print("u[");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(u[i]);
  }
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

float rotx(int theta, int wx, int wy) {
  float ftheta = -1 * theta * (PI / 180);
  float rx = (float)wx * cos(ftheta) - (float)wy * sin(ftheta);
  //  Serial.print(rx);
  //  Serial.print(",");
  return rx;
}

float roty(int theta, int wx, int wy) {
  float ftheta = -1 * theta * (PI / 180);
  float ry = (float)wx * sin (ftheta) + (float)wy * cos(ftheta);
  //  Serial.print(ry);
  //  Serial.print(",");
  return ry;
}


int x_acc300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = (now_tim - init_tim) * v;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int x_accm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = -1 * (now_tim - init_tim) * v;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_acc300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = 0;
    vy = (now_tim - init_tim) * v;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_accm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = 0;
    vy = -1 * (now_tim - init_tim) * v;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_dec300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 0;
    vy = 600 - (now_tim - init_tim - 350) * v;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    vx = 0;
    vy = 0;
    vt = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    return 0;
  }
}

int y_decm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 0;
    vy = -600 + (now_tim - init_tim - 350) * v;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    vx = 0;
    vy = 0;
    vt = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    return 0;
  }
}

int x_dec300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 600 - (now_tim - init_tim - 350) * v;
    vy = 0;
    vt = 0;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    vx = 0;
    vy = 0;
    vt = vt;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    return 0;
  }
}

int x_decm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = -600 + (now_tim - init_tim - 350) * v;
    vy = 0;
    vt = 0;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    vx = 0;
    vy = 0;
    vt = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    return 0;
  }
}

int x_strm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int x_strm150(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 500) {//150mm-定速
    vx = -600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int x_str300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int x_str100(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 333) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);

    vwx = vwx / 1.52;
    vwy = vwy / 1.52;
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_strm300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_str300(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_str150(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 500) {//150mm-定速
    vx = 0;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int turn_x2y(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = 600;
    vy = (now_tim - init_tim) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600 - (now_tim - init_tim - 350) * 2;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_mx2y(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -600;
    vy = (now_tim - init_tim) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600 - (now_tim - init_tim - 350) * 2;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_x2my(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = 600;
    vy = -1 * (now_tim - init_tim) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600 - (now_tim - init_tim - 350) * 2;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_mx2my(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -600;
    vy = -1 * (now_tim - init_tim) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
  } else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600 - (now_tim - init_tim - 350) * 2;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_y2x(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * 2;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600;
    vy = 600 - (now_tim - init_tim - 350) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_my2x(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * 2;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600;
    vy = -600 + (now_tim - init_tim - 350) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}


int turn_y2mx(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * -2;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = 600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600;
    vy = 600 - (now_tim - init_tim - 350) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_my2mx(int init_tim, int now_tim, int gyro_x) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -1 * (now_tim - init_tim) * 2;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = -600;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600;
    vy = -600 + (now_tim - init_tim - 350) * 2;
    vt = vt;

    float vwx = rotx(gyro_x, vx, vy);
    float vwy = roty(gyro_x, vx, vy);
    u[0] = (int)((vwy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vwx - 0.5 * vwy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}
