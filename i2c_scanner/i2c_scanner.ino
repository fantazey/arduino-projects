#include "Wire.h"
void setup() {
   Wire.begin();
   Serial.begin(9600);  // запуск последовательного порта
}

int devices;
byte err, addr;

void loop() {
   Serial.println("Start scan I2C bus...");
   devices = 0;
   Serial.print("  00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
   for(addr = 0; addr<= 127; addr++ ) {
      if((addr% 0x10) == 0) {
         Serial.println();
         if(addr< 16)
         Serial.print('0');
         Serial.print(addr, 16);
         Serial.print(" ");
      }
   Wire.beginTransmission(addr);err = Wire.endTransmission();
   if (err == 0) {
      if (addr<16)
         Serial.print("0");
      Serial.print(addr, HEX);
      devices++;
   }
   else {
      Serial.print("--");
   }
   Serial.print(" ");
   delay(1);
   }
   Serial.println();
   if (devices == 0)
      Serial.println("No I2C devices found\n");

   delay(6000);
}