#ifndef m3508_h
#define m3508_h
#include "pid.h"
#include <Arduino.h>

class m3508 {
  public:
    void init();
    void update(int pid0up, int pid1up, int pid2up, int gyro_x);
    void setspd(float vx, float vy, float vt, int flag);
    void pidto();
    int gyro(int gyro_x, int goaltg, float my2_yaw_org, int flag04);
    int u[4];
  private:
    float vwx, vwy;
    int flag03;
    int goal;
};

#endif
