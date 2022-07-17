#ifndef rotV_h
#define rotV_h
#include <Arduino.h>

class rotV{
  public:
    void init();
    void update();
    float setRx(float x, float y, int theta);
    float setRy(float x, float y, int theta);
  private:
    int u[4];
    float ftheta, rx, ry;
};

#endif
