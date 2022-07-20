#include <Servo.h>

Servo myservo_ver; //垂直 88(down), 100(up), 95(stop)
Servo myservo_par; //並行 87(flont), 100(back), 94(stop)

int pos = 0;    // variable to store the servo position

void setup() {
  myservo_ver.attach(22);
  myservo_par.attach(23);
}

void loop() {
  //  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
  // in steps of 1 degree
  myservo_par.write(94);
  delay(15);
  //}
}
