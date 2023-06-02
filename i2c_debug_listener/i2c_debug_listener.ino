#include <SoftwareSerial.h>

#define RX_PIN_MASTER 5 // to pb3
#define TX_PIN_MASTER 6 // to pb4
SoftwareSerial masterSerial(RX_PIN_MASTER, TX_PIN_MASTER);


#define RX_PIN 4
#define TX_PIN 3

SoftwareSerial slaveSerial(RX_PIN, TX_PIN);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  masterSerial.begin(9600);
  slaveSerial.begin(9600);
}

void loop() {  
  if (masterSerial.available()) {
    Serial.print("MASTER: ");
    while(masterSerial.available() > 0) {
      Serial.write(masterSerial.read());
    }
    Serial.println("END MASTER");
  }

if (slaveSerial.available()) {
    Serial.print("SLAVE: ");
    while(slaveSerial.available() > 0) {
      Serial.write(slaveSerial.read());
    }
    Serial.println("END SLAVE");
  }
}
