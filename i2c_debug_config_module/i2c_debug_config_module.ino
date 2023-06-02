#define RX_PIN PB4
#define TX_PIN PB3
#include <TinyWireS.h>
#include <SoftwareSerial.h>
// #include <DFRobot_DHT11.h>
#define DHT_I2C_ADDRESS 9

// DFRobot_DHT11 DHT;
// uint16_t dhtReadDelay = 10000;
// uint32_t dhtTimer = 0;

SoftwareSerial ms(RX_PIN, TX_PIN);

#define CONFIG_DATA_SIZE 32
uint8_t maxSend = 10;
uint8_t configData[32] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};


void setup() {  
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  // pinMode(DHT11_PIN, INPUT);
  ms.begin(9600);
  TinyWireS.begin(DHT_I2C_ADDRESS);
  TinyWireS.onRequest(requestEvent);
  TinyWireS.onReceive(receiveEvent);
}

void requestEvent() {
  uint8_t b1=0,b2=0;
  ms.print(F("WRITE CONFIG"));
  for (uint8_t i=maxSend;i>=0;i--) {
    ms.print(F("index: "));
    ms.println(i);
    ms.print(F(" data: "));
    ms.println(configData[i]);  
    TinyWireS.write(configData[i]);
  }
  ms.print(F("END TRANSMISSION"));
  // configData[CONFIG_DATA_IDX_CHANGED] = 0;
}

void receiveEvent() {
  uint8_t b1 = 0, b2 = 0;
  while(TinyWireS.available()>0) {
      b1 = TinyWireS.read();
      if (b1 == 120) {
        maxSend = TinyWireS.read();
      }
  }
}


void readDataUpdates() {  
  uint8_t c;
  while (ms.available() > 0) {
    c = ms.read();
    if (c == 'm') {
      maxSend = ms.parseInt();
    }
  }
}

void loop() {
  readDataUpdates();
}
