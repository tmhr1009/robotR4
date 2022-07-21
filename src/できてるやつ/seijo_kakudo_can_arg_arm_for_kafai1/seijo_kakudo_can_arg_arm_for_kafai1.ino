#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "pid.h"
#include <FastLED.h>
#include <Servo.h>

#define sin120 0.86602540378

#define NUM_LEDS 5
#define DATA_PIN 21
CRGB leds[NUM_LEDS];

Pid pid0;
Pid pid11;
Pid pid2;
Pid pidpid;
Pid pid_ang;

FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t state_msg;
static CAN_message_t ang_msg;
static CAN_message_t rxmsg;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

Servo myservo_ver; //垂直 85(down), 103(up), 97(stop)
Servo myservo_par; //並行 85(flont), 104(back), 96(stop)

// table_pos[0] テーブルホーム
int table_pos[7] = {695, 1865, 3035, 4205, 5375, 6545, 7715};
int hand__num = 0;
int cou = 0;
int ca_re = 0; // 0=(停止), 1=ca(拾う), 2=re(出す)
int flag1 = 0; // catch=ca switch用 動作終了後1になる
int place = 0; //次置く場所
int now_tgt_place = 0;
int stage__arm = 0;
int tim_t = 0;
int now_ang = 0;
int hand__task = 0;
int void_switch_num = 0;
int table_gosa = 110;//許容範囲とするテーブル誤差
int tgt_ang = table_pos[0]; //目標値
int ang_u[4] = {0};
//int arm_ver_down = 87;
//int arm_ver_up = 87;
//int par_ver_flont = 87;
//int par_ver_back = 87;


long vel[2];
int flag = 0;
float vx, vy, vt;
float v = 1.5;
int goal = 0;
int goaltg = 0;
int u[4] = {0};
int now_dir = 0;//0=北, 1=南, 2=西, 3=東
int pos_dir[4] = {0, 180, 270, 90};
int goal_dir = 0;
int state__send = 0;

void setup() {
  CANTransmitter.begin();
  pid0.init(5.1, 0.06, 0);
  pid11.init(3.8, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(150.0, 0, 1);
  pid_ang.init(7.5, 0.0, 0.02);
  myservo_ver.attach(22);//垂直？
  myservo_par.attach(23);//水平?
  pinMode(6, OUTPUT); //電磁弁
  pinMode(7, OUTPUT); //真空モータ
  Serial.begin(115200);
  msg.id = 0x200;
  msg.len = 8;
  state_msg.id = 0x0;
  state_msg.len = 2;
  state_msg.buf[0] = 0; //待機中
  state_msg.buf[1] = 0; //北向き
  ang_msg.id = 0x1FF;
  ang_msg.len = 8;
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
  ang_serial();

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
  if (!((vel[0] == 0) || (vel[1] == 0))) {
    state__send = 2;
  } else {
    state__send = 0;
  }

  Serial.println();
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

  vt = min(max(vt, -500), 500);

  u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);

  u[0] = pid0.pid_out(u[0]);
  u[1] = pid11.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);
  Serial.println();
  Serial.print("u[0] : ");
  Serial.print(u[0]);
  Serial.println();
  Serial.print("u[1] : ");
  Serial.print(u[1]);
  Serial.println();
  Serial.print("u[2] : ");
  Serial.print(u[2]);
  Serial.println();

  //  for (int i = 0; i < 4; i++)u[i] = 0;

  //テーブル，アーム制御
  if (!(hand__task == ca_re)) {
    place = 0;
    stage__arm = 0;
    ca_re = 0;
    flag1 = 0;
  }

  if (hand__task == 7) {
    myservo_ver.write(97);
    myservo_par.write(96);
    now_tgt_place = 0;
    place = 0;
    stage__arm = 0;
    ca_re = 0;
    ang_serial();
  }

  ca_re = hand__task;

  // 1=ca(拾う)
  if (ca_re == 0) {
    if (!(state_msg.buf[0] == 4)) {
      startup_arm();
    }
  } else if (ca_re == 1) {
    if (flag1 == 0) {
      switch (stage__arm) {
        case 0:
          now_tgt_place = 0;
          if (arg_spot_home(tim_t, millis()) == 0) {
            tim_t = millis();
            stage__arm++;
          }
          state__send = 3;
          break;
        case 1:
          //テーブルホームでアーム降下
          if (arm_ver_down(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 2:
          //吸盤吸い込み
          if (vac_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 3:
          if (arm_ver_up(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 4:
          //置くところまでテーブル回転
          now_tgt_place = place;
          myservo_ver.write(107);
          if (arg_move_end(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 5:
          //吸盤離す
          if (vac_release(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 6:
          flag1 = 1;
          now_tgt_place = 0;
          place = 0;
          stage__arm = 0;
          arm_stop_ver_par();
          break;
      }
    }
  } else if (ca_re == 4) {
    if (flag1 == 0) {
      switch (stage__arm) {
        case 0:
          now_tgt_place = 0;
          if (arg_spot_home(tim_t, millis()) == 0) {
            tim_t = millis();
            stage__arm++;
          }
          state__send = 3;
          break;
        case 1:
          //テーブルホームでアーム降下
          if (arm_ver_down(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 2:
          //吸盤吸い込み
          if (vac_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 3:
          if (arm_ver_up(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 4:
          //置くところまでテーブル回転
          now_tgt_place = 2;
          myservo_ver.write(107);
          if (arg_move_end_kadai1(tim_t, millis(), 2) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 5:
          //吸盤離す
          if (vac_release(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 6:
          place = 0;
          now_tgt_place = 0;
          if (arg_spot_home(tim_t, millis()) == 0) {
            tim_t = millis();
            stage__arm++;
          }
          break;
        case 7:
          if (par_flont_second(tim_t, millis()) == 0) {
            tim_t = millis();
            stage__arm++;
          }
          break;
        case 8:
          //アーム降下
          if (arm_ver_down(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 9:
          //吸盤吸い込み
          if (vac_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 10:
          if (arm_ver_up(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 11:
          now_tgt_place = 1;
          if (par_back_second(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 12:
          //置くところまでテーブル回転
          now_tgt_place = 1;
          myservo_ver.write(107);
          if (arg_move_end_kadai1(tim_t, millis(), 1) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 13:
          //吸盤離す
          if (vac_release(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 14:
          now_tgt_place = 3;
          if (par_flont_third(tim_t, millis()) == 0) {
            tim_t = millis();
            stage__arm++;
          }
          break;
        case 15:
          //アーム降下
          if (arm_ver_down(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 16:
          //吸盤吸い込み
          if (vac_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 17:
          if (arm_ver_up(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 18:
          if (par_back_third(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 19:
          //置くところまでテーブル回転
          now_tgt_place = 3;
          myservo_ver.write(107);
          if (arg_move_end_kadai1(tim_t, millis(), 3) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 20:
          //吸盤離す
          if (vac_release(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 21:
          flag1 = 1;
          now_tgt_place = 0;
          place = 0;
          stage__arm = 0;
          arm_stop_ver_par();
          break;
      }
    }
  } // 1=release(出す)
  else if (ca_re == 2) {
    if (flag1 == 0) {
      switch (stage__arm) {
        case 0:
          now_tgt_place = hand__num;
          tim_t = millis();
          stage__arm++;
          state__send = 3;
          break;
        case 1:
          //取り出すところまでテーブル回転
          place = hand__num;
          if (arg_move_end(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 2:
          //アーム降下
          if (arm_ver_down_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 3:
          //吸盤吸い込み
          if (vac_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 4:
          //アーム降下上昇
          if (arm_ver_up_pick(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 5:
          //アーム前進
          if (arm_par_flont(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 6:
          //吸盤開放 落とす
          if (vac_release(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 7:
          //アーム後退
          myservo_ver.write(110);
          if (arm_par_back(tim_t, millis()) == 0) {
            stage__arm++;
            tim_t = millis();
          }
          break;
        case 8:
          flag1 = 1;
          stage__arm = 0;
          place = 0;
          arm_stop_ver_par();
          break;
      }
    }
  }

  for (int i = 0; i < 4; i++)
    ang_u[i] = max(min(pid_ang.pid_out(tgt_ang), 1500), -1500);
  for (int i = 0; i < ang_msg.len; i++) {
    ang_msg.buf[i * 2] = ang_u[i] >> 8;
    ang_msg.buf[i * 2 + 1] = ang_u[i] & 0x00FF;
  }
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  //    state_msg.buf[0] = state__send;

  cou++;
  delay(5);
}

void timerInt() {
  tgt_ang = table_pos[now_tgt_place];
  if (hand__task == 7) {
    robo_reset();
  }
  if (state_msg.buf[0] == 4) {
    flag = 0;
  }
  else {
    flag = 1;
  }

  if (digitalRead(13) == HIGH) {
    flag1 = 0;
    state_msg.buf[0] = 4;
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    myservo_ver.write(95);
    myservo_par.write(94);
    for (int i = 0; i < msg.len; i++) {
      msg.buf[i * 2] = 0;
      msg.buf[i * 2 + 1] = 0;
    }
    for (int i = 0; i < ang_msg.len; i++) {
      ang_msg.buf[i * 2] = 0;
      ang_msg.buf[i * 2 + 1] = 0;
    }
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = CRGB(0, 255, 0);//赤
  }
  if (flag) {
    CANTransmitter.write(msg);
    CANTransmitter.write(ang_msg);
  }
  CANTransmitter.write(state_msg);
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x70) {
      vel[0] = -1 * map(rxmsg.buf[0] + rxmsg.buf[1] * 256, 0, 65535, -1023, 1023);
      vel[1] = -1 * map(rxmsg.buf[2] + rxmsg.buf[3] * 256, 0, 65535, 1023, -1023);
    }
    if (rxmsg.id == 0x71) {
      goal_dir = rxmsg.buf[0];
      hand__task = rxmsg.buf[1];
      hand__num = rxmsg.buf[2];
      place = rxmsg.buf[2];
    }
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x202) {
      pid11.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x203) {
      pid2.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
    if (rxmsg.id == 0x205) {
      int can_now_ang = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      pid_ang.now_value(can_now_ang);
      now_ang = can_now_ang;
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

void startup_arm() {
  myservo_ver.write(100);
  myservo_par.write(100);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
}

//２回目アーム前進 par_flont_second
int par_flont_second(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 1500) {
    myservo_par.write(78);
    myservo_ver.write(104);
    return 1;
  } else {
    myservo_par.write(96);
    myservo_ver.write(97);
    return 0;
  }
}

//３回目アーム前進 par_flont_third
int par_flont_third(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 2500) {
    myservo_par.write(78);
    myservo_ver.write(104);
    return 1;
  } else {
    myservo_par.write(96);
    myservo_ver.write(97);
    return 0;
  }
}

//２回目アーム後退 par_back_second
int par_back_second(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 1750) {
    myservo_par.write(113);
    return 1;
  } else {
    myservo_par.write(96);
    return 0;
  }
}

//３回目アーム後退 par_back_third
int par_back_third(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 2500) {
    myservo_par.write(113);
    return 1;
  } else {
    myservo_par.write(96);
    return 0;
  }
}

//吸い込みアーム降下 ver_down_pick
int arm_ver_down_pick(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 2100) {
    myservo_ver.write(85);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//吸い込みアーム上昇 ver_up_pick
int arm_ver_up_pick(int init_tim_t, int now_tim_t)
{
  if (now_tim_t - init_tim_t < 1800) {
    myservo_ver.write(110);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム降下 ver_down
int arm_ver_down(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 3900) { // 11000
    myservo_ver.write(78);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム上昇 ver_up
int arm_ver_up(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 4500) { // 13500
    myservo_ver.write(115);
    return 1;
  } else {
    myservo_ver.write(97);
    return 0;
  }
}

//アーム前進 par_flont
int arm_par_flont(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 4500) {
    myservo_par.write(78);
    myservo_ver.write(104);
    return 1;
  } else {
    myservo_par.write(96);
    myservo_ver.write(97);
    return 0;
  }
}

//アーム後退 par_back
int arm_par_back(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 5000) {
    myservo_par.write(113);
    return 1;
  } else {
    myservo_par.write(96);
    return 0;
  }
}

//吸盤吸い込み vac_pick
int vac_pick(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 350) {
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
int vac_release(int init_tim_t, int now_tim_t) {
  if (now_tim_t - init_tim_t < 350) {
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

int arg_move_end(int init_tim_t, int now_tim_t) {
  if ((table_pos[place] - table_gosa < now_ang) && (now_ang < table_pos[place] + table_gosa)) {
    return 0;
  } else {
    return 1;
  }
}

int arg_move_end_kadai1(int init_tim_t, int now_tim_t, int place_kadai1) {
  if ((table_pos[place_kadai1] - table_gosa < now_ang) && (now_ang < table_pos[place_kadai1] + table_gosa)) {
    return 0;
  } else {
    return 1;
  }
}

int arg_spot_home(int init_tim_t, int now_tim_t) {
  if ((table_pos[0] - table_gosa < now_ang) && (now_ang < table_pos[0] + table_gosa))
  {
    return 0;
  } else {
    return 1;
  }
}

void box__reset() {
  now_tgt_place = 0;
  place = 0;
  stage__arm = 0;
  ca_re = 0;
}

void ang_serial() {
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
  Serial.print("stage__arm: ");
  Serial.println(stage__arm);
  Serial.print("hand__num: ");
  Serial.println(hand__num);
  Serial.print("state__send: ");
  Serial.println(state__send);
  Serial.print("void int: ");
  Serial.println(millis() - tim_t);
  Serial.println();
}
