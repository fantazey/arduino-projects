#include <SoftwareSerial.h>

#define RX_PIN 4
#define TX_PIN 3

SoftwareSerial ms(RX_PIN, TX_PIN);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  ms.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    ms.write(Serial.read());
  }
}
