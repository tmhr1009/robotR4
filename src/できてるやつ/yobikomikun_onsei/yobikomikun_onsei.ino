int PINNO = 13;     // ブザーを接続したピン番号

#define NOTE_C3  131//ド
#define NOTE_CS3 139
#define NOTE_D3  147//レ
#define NOTE_DS3 156
#define NOTE_E3  165//ミ
#define NOTE_F3  175//ファ
#define NOTE_FS3 185
#define NOTE_G3  196//ソ
#define NOTE_GS3 208
#define NOTE_A3  220//ラ
#define NOTE_AS3 233
#define NOTE_B3  247//シ
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976


void setup() {
}

//呼び込み君
void loop() {
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);
  tone(PINNO, NOTE_B5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);

  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);
  tone(PINNO, NOTE_B5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);

  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 600) ;
  delay(650);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);

  tone(PINNO, NOTE_FS5, 600) ;
  delay(650);
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 800) ;
  delay(850);

  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 800) ;
  delay(850);

  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 800) ;
  delay(850);

  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 200) ;
  delay(250);
  tone(PINNO, NOTE_D5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 400) ;
  delay(450);
  tone(PINNO, NOTE_FS5, 400) ;
  delay(450);

  tone(PINNO, NOTE_A5, 400) ;
  delay(450);
  tone(PINNO, NOTE_G5, 400) ;
  delay(450);
  tone(PINNO, NOTE_FS5, 400) ;
  delay(450);
  tone(PINNO, NOTE_E5, 400);
  delay(450);


  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);
  tone(PINNO, NOTE_B5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);

  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 400) ;
  delay(450);
  tone(PINNO, NOTE_B5, 200) ;
  delay(250);
  tone(PINNO, NOTE_A5, 200) ;
  delay(250);
  tone(PINNO, NOTE_FS5, 200) ;
  delay(250);
  tone(PINNO, NOTE_E5, 400) ;
  delay(450);

  tone(PINNO, NOTE_D5, 1600) ;
  delay(1650);

}
