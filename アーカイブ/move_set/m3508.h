#ifndef m3508_h
#define m3508_h
#include "pid.h"
#include <Arduino.h>

class m3508 {
  public:
    void init();
    void update();
    void setspd(float vx, float vy, float vt, int flag);
    void pidto();
    int u[4];
    void update_pid0(int a);
    void update_pid1(int b);
    void update_pid2(int c);
  private:
    float vwx, vwy;
};

#endif
