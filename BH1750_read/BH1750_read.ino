#include <Arduino.h>
#include <Wire.h>
#include <ArtronShop_BH1750.h>

ArtronShop_BH1750 bh1750(0x23, &Wire); // Non Jump ADDR: 0x23, Jump ADDR: 0x5C

void setup() {
  Serial.begin(9600);
  Serial.println("start setup");
  Wire.begin();
  while (!bh1750.begin()) {
    Serial.println("BH1750 not found !");
    delay(1000);
  }
  Serial.println("setup ok");
}

void loop() {
  Serial.print("Light: ");
  Serial.print(bh1750.light());
  Serial.print(" lx");
  Serial.println();
  delay(6000);
}
