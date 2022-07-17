#ifndef _PID_H
#define _PID_H
#include <Arduino.h>

class MoveB {
  public:
    int pitchCommand;
    int pitchValue;
    int nowspd;

  private:
    const float pgain[3];
    static float pIe;
    static float prepPe;
    static float pPe;
};

int PIDh0(int pitchCommand, int pitchValue, int nowspd);

int PIDh1(int pitchCommand, int pitchValue, int nowspd);

int PIDh2(int pitchCommand, int pitchValue, int nowspd);


#endif
