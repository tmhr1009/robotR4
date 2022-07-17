#ifndef m3508_h
#define m3508_h
#include <Arduino.h>

class m3508 {
  public:
    void init();
    void update();
    void setspd(float vx, float vy, float vt, int gyro_x);
  private:
    int u[4];
    float vwx, vwy;
};

#endif
