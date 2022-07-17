#include <MsTimer2.h>
#include <FlexCAN.h>
#include <Servo.h>
#include <kinetis_flexcan.h>
#include "pid.h"

int led = 13;

Servo myservo_ver; //垂直 88(down), 100(up), 95(stop)
Servo myservo_par; //並行 87(flont), 100(back), 94(stop)

// create CAN object
FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

Pid pid_ang;

//pos[0] テーブルホーム
int pos[7] = {695, 1865, 3035, 4205, 5375, 6545, 7715};

int yaw_tgt_ang = pos[0]; //目標値

int cou = 0;
int inout = 0;//0=in(拾う), 1=out(出す)
int flag1 = 0;//input=in switch用
int place = 0;//次置く場所
int flag2 = 0;//input=out switch用
int stage_arm_pick = 0;
int stage_arm_rele = 0;
int tim = 0;
int now_ang = 0;

void setup() {
  Serial.begin(115200);
  CANTransmitter.begin();
  pinMode(led, OUTPUT);
  pid_ang.init(7.5, 0.0, 0.04);
  myservo_ver.attach(22);
  myservo_par.attach(23);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
  yaw_tgt_ang = pos[0];
}

int u[4] = {0};

void loop() {
  if ((yaw_tgt_ang - 50 < now_ang) && (now_ang < yaw_tgt_ang + 50)) {
    Serial.println(flag1);
    Serial.println(stage_arm_pick);
    Serial.println();

    if (place == 7) { //一周したら0に
      place = 0;
    }

    //0=in(拾う)
    if (inout == 0) {
      if (flag1 == 0) {
        switch (stage_arm_pick) {
          case 0:
            stage_arm_pick++;
            tim = millis();
            break;
          case 1:
            if (arm_ver_down(tim, millis()) == 0) {
              stage_arm_pick++;
              tim = millis();
              //この次吸盤吸い込み
            }
            break;
          case 2:
            if (arm_ver_up(tim, millis()) == 0) {
              stage_arm_pick++;
              tim = millis();
            }
            break;
          case 3:
            //置くところまでテーブル回転
            yaw_tgt_ang = pos[place];
            if (arg_move_end() == 0) {
              stage_arm_pick++;
            }
            break;
          case 4:
            flag1 = 1;
            break;
        }
      } else { //拾う→載せるまで終わったあと
        //テーブルホームに戻る
        yaw_tgt_ang = pos[0];
        //吸盤脱力
        stage_arm_pick = 0;
        flag1 = 0;
      }

    } //1=out(出す) まだ作ってないCPのみ
    else if (inout == 1) {
      if (flag2 == 0) {
        //出すところの指定
        yaw_tgt_ang = pos[place];
        switch (stage_arm_rele) {
          case 0:
            stage_arm_pick++;
            tim = millis();
            break;
          case 1:
            if (arm_ver_down(tim, millis()) == 0) {
              stage_arm_rele++;
              tim = millis();
              //この次吸盤吸い込み
            }
            break;
          case 2:
            if (arm_ver_up(tim, millis()) == 0) {
              stage_arm_rele++;
              tim = millis();
            }
            break;
          case 3:
            flag2 = 1;
            break;
        }
      } else { //出し終わったあと
        //テーブルホームに戻る
        yaw_tgt_ang = pos[0];
        //吸盤脱力
        stage_arm_rele = 0;
        flag2 = 0;
      }
    }



  }


  msg.id = 0x1FF;
  msg.len = 8;
  for (int i = 0; i < 4; i++)
    u[i] = max(min(pid_ang.pid_out(yaw_tgt_ang), 1000), -1000);
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0x00FF;
  }
  digitalWrite(led, !digitalRead(led));
  cou++;
  delay(10);
}


void timerInt() {
  CANTransmitter.write(msg);
  if (CANTransmitter.read(rxmsg) == 1 && rxmsg.id == 0x205) {
    now_ang = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    if (yaw_tgt_ang - now_ang > 4095)now_ang = now_ang + 8191;
    if (yaw_tgt_ang - now_ang < -4095)now_ang = now_ang - 8191;
    pid_ang.now_value(now_ang);
  }
}

//アーム降下 ver_down
int arm_ver_down(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) { //11000
    Serial.println(now_tim - init_tim);
    myservo_ver.write(88);
    return 1;
  } else {
    myservo_ver.write(94);
    return 0;
  }
}

//アーム上昇 ver_up
int arm_ver_up(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {//13500
    Serial.println(now_tim - init_tim);
    myservo_ver.write(102);
    return 1;
  } else {
    myservo_ver.write(94);
    return 0;
  }
}

//アーム前進 par_flont
int arm_par_flont(int init_tim, int now_tim) {
  if (now_tim - init_tim < 10000) {
    Serial.println(now_tim - init_tim);
    myservo_par.write(87);
    return 1;
  } else {
    myservo_par.write(94);
    return 0;
  }
}

//アーム後退 par_back
int arm_par_back(int init_tim, int now_tim) {
  if (now_tim - init_tim < 10000) {
    Serial.println(now_tim - init_tim);
    myservo_par.write(103);
    return 1;
  } else {
    myservo_par.write(94);
    return 0;
  }
}

int arg_move_end() {
  if ((pos[place] - 50 < now_ang) && (now_ang < pos[place] + 50)) {
    return 0;
  } else {
    return 1;
  }
}
