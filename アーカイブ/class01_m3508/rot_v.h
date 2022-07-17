#ifndef rot_v_h
#define rot_v_h
#include <Arduino.h>

class rot_v{
  public:
    void init();
    void update();
    void setRx(int x, int y, int theta);
    void setRy(int x, int y, int theta);
  private:
    int u[4];
    float ftheta, rx, ry;
};

#endif
