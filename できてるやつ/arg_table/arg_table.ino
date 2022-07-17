#include <MsTimer2.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "pid.h"

int led = 13;
// create CAN object
FlexCAN CANTransmitter(1000000);
static CAN_message_t msg;
static CAN_message_t rxmsg;

Pid pid_ang;

int pos[7] = {695, 1865, 3035, 4205, 5375, 6545, 7715};

int yaw_tgt_ang = 695; //目標値

int cou = 0;

void setup() {
  Serial.begin(115200);
  CANTransmitter.begin();
  pinMode(led, OUTPUT);
  pid_ang.init(7.0, 0.0, 0.01);
  MsTimer2::set(2, timerInt);
  MsTimer2::start();
}

int u[4] = {0};
void loop() {
  //  yaw_tgt_ang=cou%8191;
  yaw_tgt_ang = pos[0];
  msg.id = 0x1FF;
  msg.len = 8;
  for (int i = 0; i < 4; i++)
    u[i] = max(min(pid_ang.pid_out(yaw_tgt_ang), 1000), -1000);
  for (int i = 0; i < msg.len; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0x00FF;
  }
  digitalWrite(led, !digitalRead(led));
  cou++;
  delay(10);
}

void timerInt() {
  CANTransmitter.write(msg);
  if (CANTransmitter.read(rxmsg) == 1 && rxmsg.id == 0x205) {
    int now_ang = rxmsg.buf[0] * 256 + rxmsg.buf[1];
    if (yaw_tgt_ang - now_ang > 4095)now_ang = now_ang + 8191;
    if (yaw_tgt_ang - now_ang < -4095)now_ang = now_ang - 8191;
    pid_ang.now_value(now_ang);
    Serial.print(yaw_tgt_ang);
    Serial.print(",");
    Serial.println(now_ang);
  }
}
