#include <SoftwareSerial.h>

SoftwareSerial btSerial(3, 4);
/*
   Connect pin 2 Arduino to pin TX HC-06
   Connect pin 3 Arduino to pin RX HC-06
*/
// 00:1A:7D:DA:71:05
void setup() {
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  btSerial.begin(9600);
}

void loop()
{
  if (btSerial.available())
    Serial.write(btSerial.read());
  if (Serial.available())
    btSerial.write(Serial.read());
}