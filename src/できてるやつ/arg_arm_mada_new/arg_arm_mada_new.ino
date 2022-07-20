#include <MsTimer2.h>
#include <FlexCAN.h>
#include <Servo.h>
#include <kinetis_flexcan.h>
#include "pid.h"

int led = 13;

Servo myservo_ver; //垂直 85(down), 103(up), 97(stop)
Servo myservo_par; //並行 85(flont), 104(back), 96(stop)

// create CAN object
FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

Pid pid_ang;

// table_pos[0] テーブルホーム
int table_pos[7] = {695, 1865, 3035, 4205, 5375, 6545, 7715};

int hand_num = 0;
int cou = 0;
int ca_re = 0; // 0=(停止), 1=ca(拾う), 2=re(出す)
int flag1 = 0; // catch=ca switch用 動作終了後1になる
int place = 0; //次置く場所
int now_tgt_place = 0;
int stage_arm = 0;
int tim = 0;
int now_ang = 0;
int hand__task = 0;
int void_switch_num = 0;
int table_gosa = 100; //許容範囲とするテーブル誤差
int tgt_ang = table_pos[0]; //目標値

int u[4] = {0};

void setup()
{
  Serial.begin(115200);
  CANTransmitter.begin();
  pinMode(led, OUTPUT);
  pid_ang.init(7.5, 0.0, 0.02);
  myservo_ver.attach(14);
  myservo_par.attach(15);
  pinMode(6, OUTPUT); //電磁弁
  pinMode(7, OUTPUT); //真空モータ
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
}

void loop()
{
  Serial.println();
  Serial.print("ca_re: ");
  Serial.println(ca_re);
  Serial.print("tgt_ang: ");
  Serial.println(tgt_ang);
  Serial.print("now_ang: ");
  Serial.println(now_ang);
  Serial.print("place: ");
  Serial.println(place);
  Serial.print("hand__task: ");
  Serial.println(hand__task);
  Serial.print("flag1: ");
  Serial.println(flag1);
  Serial.print("now_tgt_place: ");
  Serial.println(now_tgt_place);
  Serial.print("stage_arm: ");
  Serial.println(stage_arm);
  Serial.print("hand_num: ");
  Serial.println(hand_num);
  Serial.print("void int: ");
  Serial.println(millis() - tim);
  Serial.println();

  if (!(hand__task == ca_re)) flag1 = 0;
  ca_re = hand__task;

  // 0=ca(拾う)
  if (ca_re == 1) {
    if (flag1 == 0) {
      switch (stage_arm) {
        case 0:
          now_tgt_place = 0;
          tim = millis();
          stage_arm++;
          break;
        case 1:
          //テーブルホームでアーム降下
          if (arm_ver_down(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 2:
          //吸盤吸い込み
          if (vac_pick(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 3:
          if (arm_ver_up(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 4:
          //置くところまでテーブル回転
          now_tgt_place = place;
          myservo_ver.write(107);
          if (arg_move_end(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 5:
          //吸盤離す
          if (vac_release(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 6:
          flag1 = 1;
          now_tgt_place = 0;
          stage_arm = 0;
          arm_stop_ver_par();
          break;
      }
    }
  } // 1=release(出す)
  else if (ca_re == 2) {
    if (flag1 == 0) {
      switch (stage_arm) {
        case 0:
          now_tgt_place = hand_num;
          tim = millis();
          stage_arm++;
          break;
        case 1:
          //取り出すところまでテーブル回転
          place = hand_num;
          if (arg_move_end(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 2:
          //アーム降下
          if (arm_ver_down_pick(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 3:
          //吸盤吸い込み
          if (vac_pick(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 4:
          //アーム降下上昇
          if (arm_ver_up_pick(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 5:
          //アーム前進
          if (arm_par_flont(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 6:
          //吸盤開放 落とす
          if (vac_release(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 7:
          //アーム後退
          myservo_ver.write(110);
          if (arm_par_back(tim, millis()) == 0) {
            stage_arm++;
            tim = millis();
          }
          break;
        case 8:
          flag1 = 1;
          now_tgt_place = 0;
          stage_arm = 0;
          arm_stop_ver_par();
          break;
      }
    }
  }
  Serial.print("u[0,1,2,3]: ");
  Serial.print(u[0]);
  Serial.print(", ");
  Serial.print(u[1]);
  Serial.print(", ");
  Serial.print(u[2]);
  Serial.print(", ");
  Serial.println(u[3]);

  msg.id = 0x1FF;
  msg.len = 8;
  for (int i = 0; i < 4; i++)
    u[i] = max(min(pid_ang.pid_out(tgt_ang), 1000), -1000);
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0x00FF;
  }
  digitalWrite(led, !digitalRead(led));
  cou++;
  //  myservo_par.write(93);
  delay(5);
}

//TimerInt
void timerInt() {
  tgt_ang = table_pos[now_tgt_place];
  CANTransmitter.write(msg);
  while (CANTransmitter.read(rxmsg)) {
    if (rxmsg.id == 0x205) {
      int can_now_ang = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      pid_ang.now_value(can_now_ang);
      now_ang = can_now_ang;
    }
    if (rxmsg.id == 0x71) {
      hand__task = rxmsg.buf[1];
      hand_num = rxmsg.buf[2];
      place = rxmsg.buf[2];
    }
  }
}

//吸い込みアーム降下 ver_down_pick
int arm_ver_down_pick(int init_tim, int now_tim) {
  if (now_tim - init_tim < 2500) {
    myservo_ver.write(85);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//吸い込みアーム上昇 ver_up_pick
int arm_ver_up_pick(int init_tim, int now_tim)
{
  if (now_tim - init_tim < 3500) {
    myservo_ver.write(109);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム降下 ver_down
int arm_ver_down(int init_tim, int now_tim) {
  if (now_tim - init_tim < 8000) { // 11000
    myservo_ver.write(85);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム上昇 ver_up
int arm_ver_up(int init_tim, int now_tim) {

  if (now_tim - init_tim < 11000) { // 13500
    myservo_ver.write(109);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム前進 par_flont
int arm_par_flont(int init_tim, int now_tim) {
  if (now_tim - init_tim < 4000) {
    myservo_par.write(84);
    myservo_ver.write(104);
    return 1;
  } else if (now_tim - init_tim < 10000) {
    myservo_par.write(84);
    myservo_ver.write(94);
    return 1;
  } else {
    myservo_par.write(96);
    myservo_ver.write(97);
    return 0;
  }
}

//アーム後退 par_back
int arm_par_back(int init_tim, int now_tim) {
  if (now_tim - init_tim < 10000) {
    myservo_par.write(106);
    return 1;
  } else {
    myservo_par.write(96);
    return 0;
  }
}

//吸盤吸い込み vac_pick
int vac_pick(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    return 1;
  } else {
    //    digitalWrite(6, LOW);
    //    digitalWrite(7, LOW);
    return 0;
  }
}

//吸盤脱力 vac_release
int vac_release(int init_tim, int now_tim) {
  if (now_tim - init_tim < 1000) {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    return 1;
  } else {
    digitalWrite(6, LOW);
    digitalWrite(7, LOW);
    return 0;
  }
}

void arm_stop_ver_par() {
  myservo_ver.write(97);
  myservo_par.write(96);
}

int arg_move_end(int init_tim, int now_tim) {
  if ((table_pos[place] - table_gosa < now_ang) && (now_ang < table_pos[place] + table_gosa)) {
    return 0;
  } else {
    return 1;
  }
}

int arg_spot_home(int init_tim, int now_tim) {
  if ((table_pos[0] - table_gosa < now_ang) && (now_ang < table_pos[0] + table_gosa))
  {
    return 0;
  } else {
    return 1;
  }
}
