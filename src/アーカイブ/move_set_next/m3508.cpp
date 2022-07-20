#include "m3508.h"
#include "pid.h"
#include <Arduino.h>

#define sin120 0.86602540378

Pid pid0;
Pid pid1;
Pid pid2;
Pid pidpid;

void m3508::init() {
  u[4] = {};
  vwx = 0;
  vwy = 0;
  flag03 = 0;
  goal = 0;
  pid0.init(4.7, 0.06, 0);
  pid1.init(3.4, 0.04, 0);
  pid2.init(6.2, 0.05, 0);
  pidpid.init(150.0, 0, 1);
}

//現在地を更新
void m3508::update(int pid0up, int pid1up, int pid2up, int gyro_x) {
  pid0.now_value(pid0up);
  pid1.now_value(pid1up);
  pid2.now_value(pid2up);
  pidpid.now_value(gyro_x);
}

//モータ速度設定
//flag = 1でモーター停止
void m3508::setspd(float vx, float vy, float vt, int flag) {
  u[0] = (int)((vy + vt * 17 / 100) / 0.27557830294);
  u[1] = (int)((sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);
  u[2] = (int)((-sin120 * vx - 0.5 * vy + vt * 17 / 100) / 0.27557830294);

  if (flag == 1) {
    for (int i = 0; i < 4; i++)u[i] = 0;
  }
}

//モータPID読み込み
void m3508::pidto() {
  u[0] = pid0.pid_out(u[0]);
  u[1] = pid1.pid_out(u[1]);
  u[2] = pid2.pid_out(u[2]);
}

//角度補正PID
//flag = 1でmy_yaw_org * 100
int m3508::gyro(int gyro_x, int goal_tg, float my_yaw_org, int flag04) {
  if (flag04 == 1) {
    float my2_yaw_org = my_yaw_org * 100;
    if (goal_tg - my2_yaw_org > 36000) goal_tg = goal_tg - my2_yaw_org - 36000;
    if (goal_tg - my2_yaw_org < 0) goal_tg = goal_tg - my2_yaw_org + 36000;
    if (goal_tg - my2_yaw_org / 100 - gyro_x > 180) {
      goal = pidpid.pid_out(goal_tg - my2_yaw_org / 100 - 360);
    }
    else if (goal_tg / 100 - gyro_x < -180) {
      goal = pidpid.pid_out(goal_tg - my2_yaw_org / 100 + 360);
    }
    else {
      goal = pidpid.pid_out(goal_tg - my2_yaw_org / 100);
    }
    goal = goal / 100;
  }

  if (flag04 == 0) {
    if (goal_tg - my_yaw_org > 900) goal_tg = goal_tg - my_yaw_org / 100;
    if (goal_tg - my_yaw_org > 360) goal_tg = goal_tg - my_yaw_org - 360;
    if (goal_tg - my_yaw_org < 0) goal_tg = goal_tg - my_yaw_org + 360;
    if (goal_tg - my_yaw_org - gyro_x > 180) {
      goal = pidpid.pid_out(goal_tg - my_yaw_org - 360);
    }
    else if (goal_tg - gyro_x < -180) {
      goal = pidpid.pid_out(goal_tg - my_yaw_org + 360);
    }
    else {
      goal = pidpid.pid_out(goal_tg - my_yaw_org);
    }
  }

  pidpid.debug();
  return goal;
}
