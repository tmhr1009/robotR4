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
int wx = 0;
int wy = 0;
int sita = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
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
  int gyro_x = euler.x();
  pidpid.now_value(gyro_x);
  Serial.print(gyro_x);
  Serial.println(",");

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
  if (((data[5] & 0xC0) >> 6) == 1) {
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

    switch (stage) {
      case 0:
        //        win_start();
        stage++;
        tim = millis();
        break;
      case 1:
        if (x_acc300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 2:
        if (x_dec300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 3:
        if (y_acc300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 4:
        if (y_dec300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 5:
        if (x_acc300(tim, millis()) == 0) {
          stage++;
          tim = millis();
          goaltg = goaltg - 90;
        }
        break;
      case 6:
        if (x_dec300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 7:
        if (y_accm300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 8:
        if (y_decm300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 9:
        if (y_acc300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 10:
        if (y_str300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 11:
        if (y_str150(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 12:
        if (y_dec300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 13:
        if (x_accm300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 14:
        if (x_strm300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 15:
        if (x_decm300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 16:
        if (y_acc300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 17:
        if (y_dec300(tim, millis()) == 0) {
          stage++;
          tim = millis();
        }
        break;
      case 18:
        vx = 0;
        vy = 0;
        vt = 0;
        u[0] = 0;
        u[1] = 0;
        u[2] = 0;
        break;
    }
  }
  else if (((data[5] & 0xC0) >> 6) == 2) {//停止(フリーではない)
    stage = 0;
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
    goaltg = gyro_x * 100;
    goal = 0;
  }
  else if (((data[5] & 0xC0) >> 6) == 3) {//マニュアル操作
    if (goaltg > 36000)goaltg = goaltg - 36000;
    if (goaltg < 0)goaltg = goaltg + 36000;
    if (goaltg / 100 - gyro_x > 180) {
      goal = pidpid.pid_out(goaltg / 100 - 360);
    }
    else if (goaltg / 100 - gyro_x < -180) {
      goal = pidpid.pid_out(goaltg / 100 + 360);
    }
    else {
      goal = pidpid.pid_out(goaltg / 100);
    }
    pidpid.debug();
    vt = goal;

    //    Serial.print("角出力: ");
    //    Serial.println(goal);
    //    Serial.print("目標角: ");
    Serial.println(goaltg / 100);

    stage = 0;
    vx = map(testch[3], 364, 1684, -2000, 2000);//目標最大rpmは5000
    vy = map(testch[2], 364, 1684, -2000, 2000);
    goaltg = goaltg + map(testch[0] - 6, 364, 1684, -100, 100);
    if (abs(vt) < 5)vt = 0;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  }

  //  Serial.print(vx);
  //  Serial.print(",");
  //  Serial.print(vy);
  //  Serial.print(",");
  //  Serial.println(vt);

  u[0] = rotx(goaltg, u[0], u[1]);
  u[1] = roty(goaltg, u[0], u[1]);

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

int rotx(int sita, int wx, int wy) {
  int rx;
  sita = sita / (180 / PI);
  rx = wx * cos(sita) - wy * sin(sita);
  return rx;
}

int roty(int sita, int wx, int wy) {
  int ry;
  sita = sita / (180 / PI);
  ry = wx * cos(sita) + wy * sin(sita);
  return ry;
}


int x_acc300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = (now_tim - init_tim) * v;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int x_accm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = -1 * (now_tim - init_tim) * v;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_acc300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = 0;
    vy = (now_tim - init_tim) * v;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_accm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//300mm-加速
    vx = 0;
    vy = -1 * (now_tim - init_tim) * v;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int y_dec300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 0;
    vy = 600 - (now_tim - init_tim - 350) * v;
    vt = vt;
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

int y_decm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 0;
    vy = -600 + (now_tim - init_tim - 350) * v;
    vt = vt;
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

int x_dec300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = 600 - (now_tim - init_tim - 350) * v;
    vy = 0;
    vt = 0;
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

int x_decm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 350) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//300mm-減速
    vx = -600 + (now_tim - init_tim - 350) * v;
    vy = 0;
    vt = 0;
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

int x_strm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = -600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int x_strm150(int init_tim, int now_tim) {
  if (now_tim - init_tim < 500) {//150mm-定速
    vx = -600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int x_str300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 600;
    vy = 0;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_strm300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 0;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_str300(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {//300mm-定速
    vx = 0;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int y_str150(int init_tim, int now_tim) {
  if (now_tim - init_tim < 500) {//150mm-定速
    vx = 0;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  } else {
    return 0;
  }
}

int turn_x2y(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = 600;
    vy = (now_tim - init_tim) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600 - (now_tim - init_tim - 350) * 2;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_mx2y(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -600;
    vy = (now_tim - init_tim) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600 - (now_tim - init_tim - 350) * 2;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_x2my(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = 600;
    vy = -1 * (now_tim - init_tim) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600 - (now_tim - init_tim - 350) * 2;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_mx2my(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -600;
    vy = -1 * (now_tim - init_tim) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  } else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600 - (now_tim - init_tim - 350) * 2;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_y2x(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * 2;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600;
    vy = 600 - (now_tim - init_tim - 350) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_my2x(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * 2;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = 600;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = 600;
    vy = -600 + (now_tim - init_tim - 350) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}


int turn_y2mx(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = (now_tim - init_tim) * -2;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = 600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600;
    vy = 600 - (now_tim - init_tim - 350) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}

int turn_my2mx(int init_tim, int now_tim) {
  if (now_tim - init_tim < 300) {//600mm-左曲がり
    vx = -1 * (now_tim - init_tim) * 2;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 350) {//600mm-左曲がり
    vx = -600;
    vy = -600;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else if (now_tim - init_tim < 650) {//600mm-左曲がり
    vx = -600;
    vy = -600 + (now_tim - init_tim - 350) * 2;
    vt = vt;
    u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
    return 1;
  }
  else {
    return 0;
  }
}
