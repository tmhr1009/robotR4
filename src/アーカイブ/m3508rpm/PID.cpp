#include "Arduino.h"
#include "PID.h"


//PID
int PIDh0(int pitchCommand, int pitchValue, int nowspd) {
  const float pgain[3] = {0.8, 0, 5};
  static float pIe, prepPe;
  float pPe = pitchCommand - pitchValue;
  pIe += pPe;
  float pDe = pPe - prepPe;
  prepPe = pPe;
  if (pitchCommand == 0 || pitchValue == 0) {
    pPe = 0;
    pIe = 0;
    pDe = 0;
  }
  if (!(nowspd > 0)) {
    pIe = 0.01;
  }
  return pgain[0] * pPe + pgain[1] * pIe + pgain[2] * pDe;
}

int PIDh1(int pitchCommand, int pitchValue, int nowspd) {
  const float pgain[3] = {0.8, 0, 5};
  static float pIe, prepPe;
  float pPe = pitchCommand - pitchValue;
  pIe += pPe;
  float pDe = pPe - prepPe;
  prepPe = pPe;
  if (pitchCommand == 0 || pitchValue == 0) {
    pPe = 0;
    pIe = 0;
    pDe = 0;
  }
  if (!(nowspd > 0)) {
    pIe = 0.01;
  }
  return pgain[0] * pPe + pgain[1] * pIe + pgain[2] * pDe;
}

int PIDh2(int pitchCommand, int pitchValue, int nowspd) {
  const float pgain[3] = {0.8, 0, 5};
  static float pIe, prepPe;
  float pPe = pitchCommand - pitchValue;
  pIe += pPe;
  float pDe = pPe - prepPe;
  prepPe = pPe;
  if (pitchCommand == 0 || pitchValue == 0) {
    pPe = 0;
    pIe = 0;
    pDe = 0;
  }
  if (!(nowspd > 0)) {
    pIe = 0.01;
  }
  return pgain[0] * pPe + pgain[1] * pIe + pgain[2] * pDe;
}
