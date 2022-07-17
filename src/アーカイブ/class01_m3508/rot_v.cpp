#include "rot_v.h"
#include <Arduino.h>

void rot_v::init() {
  ftheta = 0;
  rx = 0;
  ry = 0;
}

void rot_v::update() {

}

void rot_v::setRx(int x, int y, int theta) {
  ftheta = -1 * theta * (PI / 180);
  rx = (float)x * cos(ftheta) - (float)y * sin(ftheta);
  return rx;
}

void rot_v::setRy(int x, int y, int theta) {
  ftheta = -1 * theta * (PI / 180);
  ry = (float)x * sin (ftheta) + (float)y * cos(ftheta);
  return ry;
}
