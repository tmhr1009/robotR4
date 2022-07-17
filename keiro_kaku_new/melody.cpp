#include "melody.h"
#include <Arduino.h>

void win_start() {
  //♭ミ(高）♭ミ(低）♭シ♭ラ♭ミ(低）♭ミ(高）♭シ
  tone(PINNO, NOTE_DS5, 338) ;
  delay(338 + 38);
  tone(PINNO, NOTE_DS4, 113) ;
  delay(113 + 38);
  tone(PINNO, NOTE_AS4, 225) ;
  delay(263);
  tone(PINNO, NOTE_GS4, 225) ;
  delay(263);
  tone(PINNO, NOTE_DS4, 225) ;
  delay(263);
  tone(PINNO, NOTE_DS5, 225) ;
  delay(263);
  tone(PINNO, NOTE_AS4, 338) ;
  delay(338 + 38);
}

void win_end() {
  //♭ラ(高)♭ミ(低）♭ラ（低）♭シ（低）
  tone(PINNO, NOTE_GS5, 225) ;
  delay(263);
  tone(PINNO, NOTE_DS5, 225) ;
  delay(263);
  tone(PINNO, NOTE_GS4, 225) ;
  delay(263);
  tone(PINNO, NOTE_AS4, 338) ;
  delay(338 + 38);
}

void ele() {
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_C6, 75) ;
  delay(113);
  tone(PINNO, NOTE_D6, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_E5 , 225) ;
  delay(263);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_C6, 75) ;
  delay(113);
  tone(PINNO, NOTE_D6, 225) ;
  delay(263);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
  tone(PINNO, NOTE_E5 , 225) ;
  delay(263);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_B5, 75) ;
  delay(113);
  tone(PINNO, NOTE_A5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_E5 , 75) ;
  delay(113);
  tone(PINNO, NOTE_FS5, 75) ;
  delay(113);
  tone(PINNO, NOTE_D5, 75) ;
  delay(113);
  tone(PINNO, NOTE_G5, 225) ;
  delay(263);
}
