#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "pid.h"
#include "melody.h"

#define sin120 0.86602540378

Pid pid0;
Pid pid1;
Pid pid2;

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

int tim = 0;
int flag = 0;

void setup() {
  CANTransmitter.begin();
  pinMode(13, OUTPUT);
  //  ele();
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E1);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
}

static unsigned long testch[6];

void loop() {
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
  int u[4] = {};
  msg.id = 0x200;
  msg.len = 8;
  for (int i = 0; i < 4; i++) {
    u[i] = 0;
  }
  if (((data[5] & 0xC0) >> 6) == 1) {
    if (millis() - tim < 500) {
      for (int i = 0; i < 4; i++) {
        u[i] = 0;
      }
      //      u[0] = (millis() - tim) * 4;
      //      u[1] = (millis() - tim) * 4;
      //      u[2] = (millis() - tim) * 4;
    }
    else if (millis() - tim < 5500) {
      float vx, vy, vt;
      //      u[0] = 2000;
      //      u[1] = 2000;
      //      u[2] = 2000;
      vx = 0;//目標最大rpmは5000
      vy = 200;
      vt = 0;
      u[0] = (int)((vy + vt) / 0.27557830294);
      u[1] = (int)((sin120 * vx - 0.5 * vy + vt) / 0.27557830294);
      u[2] = (int)((-sin120 * vx - 0.5 * vy + vt) / 0.27557830294);
    }
    else if (millis() - tim < 2000) {
      for (int i = 0; i < 4; i++) {
        u[i] = 0;
      }
      //      u[0] = 8000 - (millis() - tim) * 4;
      //      u[1] = 8000 - (millis() - tim) * 4;
      //      u[2] = 8000 - (millis() - tim) * 4;
    }
    else {
      u[0] = 0;
      u[1] = 0;
      u[2] = 0;
    }
  }
  else if (((data[5] & 0xC0) >> 6) == 2) {
    tim = millis();
    u[0] = 0;
    u[1] = 0;
    u[2] = 0;
  }
  else if (((data[5] & 0xC0) >> 6) == 3) {
    tim = millis();
    float vx, vy, vt;
    vx = map(testch[3], 364, 1684, -5000, 5000);//目標最大rpmは5000
    vy = map(testch[2], 364, 1684, -5000, 5000);
    vt = map(testch[0] - 6, 364, 1684, -2500, 2500);
    u[0] = (int)((vy + vt) / 0.27557830294);
    u[1] = (int)((sin120 * vx - 0.5 * vy + vt) / 0.27557830294);
    u[2] = (int)((-sin120 * vx - 0.5 * vy + vt) / 0.27557830294);
    //    u[0] = map(testch[3], 364, 1684, -1000, 1000);//目標最大rpmは5000
    //    u[1] = map(testch[0], 364, 1684, -1000, 1000) - 12;
    //    u[2] = map(testch[2], 364, 1684, -1000, 1000);
    Serial.print(vx);
    Serial.print(",");
    Serial.print(vy);
    Serial.print(",");
    Serial.print(vt);
    Serial.print(",");
    Serial.print(pid0.debug());
    Serial.print(",");
    Serial.print(pid1.debug());
    Serial.print(",");
    Serial.print(pid2.debug());
    Serial.println("");
  }
  //  Serial.print(u[1]);
  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  for (int i = 0; i < 4; i++)u[i] = 0;
  //  Serial.print(",");
  //  Serial.print(pid0.debug());
  //  Serial.print(",");
  //  Serial.print(pid1.debug());
  //  Serial.print(",");
  //  Serial.print(pid2.debug());
  //  Serial.println("");
  delay(10);
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

void ele() {
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_C6, 75) ;
  delay(113);
  tone(PINNO, NOTE_D6, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_E5 , 225) ;
  delay(263);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_C6, 75) ;
  delay(113);
  tone(PINNO, NOTE_D6, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_E5 , 225) ;
  delay(263);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
}
