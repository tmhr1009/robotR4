#include "rotV.h"
#include <Arduino.h>

void rotV::init() {
  ftheta = 0;
  rx = 0;
  ry = 0;
}

void rotV::update() {

}

float rotV::setRx(float x, float y, int theta) {
  ftheta = -1 * theta * (PI / 180);
  rx = (float)x * cos(ftheta) - (float)y * sin(ftheta);
  return rx;
}

float rotV::setRy(float x, float y, int theta) {
  ftheta = -1 * theta * (PI / 180);
  ry = (float)x * sin (ftheta) + (float)y * cos(ftheta);
  return ry;
}
