void setup() {
  // put your setup code here, to run once:
  pinMode(6, OUTPUT); //電磁弁
  pinMode(7, OUTPUT); //真空モータ
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  delay(2000);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  delay(2000);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  delay(2000);
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);

}
