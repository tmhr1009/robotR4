#include <Metro.h>
#include <FlexCAN.h>
#include <MsTimer2.h>
#include "PID.h"

typedef struct
{
  int16_t rotation;
  int16_t denryu;
} wheelEscDataSt;

Metro sysTimer = Metro(1);      // milliseconds
FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;
unsigned int txTimer;
unsigned int rxTimer;
wheelEscDataSt wEscData[4];   //WheelESCのCANデータ



Metro debugTimer = Metro(100);      // デバッグ情報出力間隔(ms)
CAN_message_t msg;

int id = 0;
int can = 0;
int nov[3] = {0, 0, 0}; //常時回転速度
int v[3] = {0, 0, 0}; //CAN送信する最終速度 CAN0,1,2
float vm[3] = {0, 0, 0}; //タイヤごとの速度 xyθ入力
float vmt[3] = {0, 0, 0}; //なし
int x = 0;
int y = 0;
int z = 0;
int count = 0;

void setup(void)
{
  CANTransmitter.begin();

  Serial.begin(115200);                 //シリアルモニタ
  Serial1.begin(100000, SERIAL_8E1);    //DJI受信機
  delay(1000);

  msg.len = 8;
  msg.id = 0x200; //走行用モーターの制御用CAN通信
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  sysTimer.reset();
  debugTimer.reset();
  MsTimer2::set(2, timerInt);
  MsTimer2::start();

}

void loop(void)
{
  // CAN通信用のタイマー(カウンタ)カウントダウン
  if ( sysTimer.check() ) {
    --txTimer;
    --rxTimer;
  }

  //タイヤごとの速度 XYθ入力
  vm[0] = 1 / (PI * PI * PI) * 1.2 * x + 0 * y + 1 / (PI * PI * PI) * 102 * z;
  vm[1] = 1 / (PI * PI * PI) * -1.29282032 * x + 1 / (PI * PI * PI) * 0.346410162 * y + 1 / (PI * PI * PI) * -58.889725 * z;
  vm[2] = 1 / (PI * PI * PI) * 0.0928203230 * x + 1 / (PI * PI * PI) * -0.346410162 * y + 1 / (PI * PI * PI) * 58.8897275 * z;


//  if (count >= 50000) {
//    for (int i = 0; i < 3; i++) {
//      vm[i] = 0;
//    }
//  }
  count++;


  if (can) {
    for (int i = 0; i < 3; i++) {
      switch (i) {
        case 0:
          v[i] = PIDh0((int)nov[i], wEscData[i].rotation - 300, wEscData[i].rotation);
          break;
        case 1:
          v[i] = PIDh1((int)nov[i], wEscData[i].rotation - 300, wEscData[i].rotation);
          break;
        case 2:
          v[i] = PIDh2((int)nov[i], wEscData[i].rotation - 300, wEscData[i].rotation);
          break;
      }

      vm[i] = vm[i] * 15;
      v[i] = vm[i] + v[i]; //PIDとXY移動指示を足す
    }
    Serial.print(nov[0]);
    Serial.print(", ");
        Serial.print(v[0]);
        Serial.print(", ");
    //    Serial.print(v[1]);
    //    Serial.print(", ");
    //    Serial.print(v[2]);
    //    Serial.print(",      ");
    Serial.print(vm[0]);
    Serial.print(", ");
//    Serial.print(vm[1]);
//    Serial.print(", ");
//    Serial.print(vm[2]);
//    Serial.print(",  ");
    //    Serial.print(wEscData[0].rotation);
//    Serial.print(count);
    Serial.println(", ");
  } else {
    PIDh0(0, 0, 0);
    PIDh1(0, 0, 0);
    PIDh2(0, 0, 0);
    Serial.println("Can Not");
  }

  int u[4] = {v[0], v[1], v[2], 0};
  for (int i = 0; i < 4; i++) {
    //u[i] = max(-1000, min(1000, u[i]));
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
}

void timerInt() {
  static int canCnt;
  while ( CANTransmitter.read(rxmsg) ) {
    switch (rxmsg.id) {
      case 0x201: // Wheel-ID1
      case 0x202: // Wheel-ID2
      case 0x203: // Wheel-ID3
      case 0x204: // Wheel-ID4
        id = rxmsg.id - 0x201;  //識別子から添え字に変換
        wEscData[id].rotation = rxmsg.buf[2] * 256 + rxmsg.buf[3];
        wEscData[id].denryu = rxmsg.buf[4] * 256 + rxmsg.buf[5];
        canCnt = 0;
        break;
    }
  }
  if (canCnt == 0) {  //正常受信
    can = 1;
    canCnt += 1;
  } else if (canCnt < 16) { //エラーだが許容範囲
    canCnt += 1;
  } else {
    can = 0;
  }

  //デバッグ情報出力タイミング
  if ( debugTimer.check()) {
    id = 0;
    for (id = 0; id < 4; id++) {
      //Serial.print(wEscData[id].rotation);
      //Serial.print(",");
      //Serial.print(wEscData[id].denryu);
      //Serial.print(",");
    }
    //Serial.println();
  }
  CANTransmitter.write(msg);
}
