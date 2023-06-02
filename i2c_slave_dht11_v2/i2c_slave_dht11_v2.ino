#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobot_DHT11.h>

#define DHT_I2C_ADDRESS 0x30

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DHT11_PIN PB1    // dht data pin
#define RX_PIN PB3
#define TX_PIN PB4
#else
#define DHT11_PIN 2       // dht data pin
#define RX_PIN 6       // software serial rx pin
#define TX_PIN 7       // software serial tx pin
#endif

#define IDX_CHANGED 0
// first block 
#define IDX_YEAR 1
#define IDX_MONTH 2
#define IDX_DAY 3
#define IDX_HOUR 4
#define IDX_MINUTE 5
#define IDX_SECOND 6
// env
#define IDX_DHT_HUMIDITY 7
#define IDX_DHT_TEMPERATURE 8

// second block - vars
// minis
#define IDX_PAINTED_H 1
#define IDX_PAINTED_L 2
#define IDX_UNPAINTED_H 3
#define IDX_UNPAINTED_L 4
// light
#define IDX_LIGHT_MODE 5
#define IDX_LIGHT_LUX_MAX_VALUE 6
#define IDX_DISPLAY_MODE 7

// 
#define BLOCK_SIZE 10
uint8_t configDataB1[BLOCK_SIZE];
uint8_t configDataB2[BLOCK_SIZE];

uint8_t blockToSend = 1;

DFRobot_DHT11 DHT;
const uint16_t dhtReadDelay = 10000;
uint32_t dhtTimer = 0;

SoftwareSerial ms(RX_PIN, TX_PIN);

void setup() {
  ms.begin(9600);
  pinMode(DHT11_PIN, INPUT);
  Wire.begin(DHT_I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  configDataB2[IDX_PAINTED_H] = EEPROM.read(IDX_PAINTED_H);
  configDataB2[IDX_PAINTED_L] = EEPROM.read(IDX_PAINTED_L);
  configDataB2[IDX_UNPAINTED_H] = EEPROM.read(IDX_UNPAINTED_H);
  configDataB2[IDX_UNPAINTED_L] = EEPROM.read(IDX_UNPAINTED_L);
  configDataB2[IDX_LIGHT_MODE] = EEPROM.read(IDX_LIGHT_MODE);
  configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = EEPROM.read(IDX_LIGHT_LUX_MAX_VALUE);
  configDataB2[IDX_DISPLAY_MODE] = EEPROM.read(IDX_DISPLAY_MODE);  
}

void requestEvent() {
  uint8_t* configBlock = blockToSend == 1 ? configDataB1 : configDataB2;
  Wire.write(250); // control byte begin of data
  for (uint8_t i=0;i<BLOCK_SIZE;i++) {
    Wire.write(configBlock[i]);
  }
  Wire.write(251); // control byte end of data
  configBlock[IDX_CHANGED] = 0;
}

void receiveEvent() {
  uint8_t c;
  if (Wire.available() > 0) {
    c = Wire.read();
    if (c == 250) {
      blockToSend = Wire.read();
    }
  }
}

void loop() {
  if (millis() - dhtTimer > dhtReadDelay) {
    DHT.read(DHT11_PIN);
    configDataB2[IDX_DHT_HUMIDITY] = DHT.humidity;
    configDataB2[IDX_DHT_TEMPERATURE] = DHT.temperature;
    dhtTimer = millis();
  }
  readDataUpdates();
}

void readDataUpdates() {  
  uint8_t c;
  uint16_t n = 0;
  while (ms.available() > 0) {
    c = ms.read();
    if (c == 'H') {
      configDataB1[IDX_HOUR] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'M') {
      configDataB1[IDX_MINUTE] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'S') {
      configDataB1[IDX_SECOND] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'y') {
      configDataB1[IDX_YEAR] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'm') {
      configDataB1[IDX_MONTH] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'd') {
      configDataB1[IDX_DAY] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    //==========================================================
    if (c == 'A') {
      configDataB2[IDX_DISPLAY_MODE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_DISPLAY_MODE, configDataB2[IDX_DISPLAY_MODE]);      
    }
    if (c == 'L') {
      configDataB2[IDX_LIGHT_MODE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_LIGHT_MODE, configDataB2[IDX_LIGHT_MODE]);
    }
    if (c == 'l') {
      configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_LIGHT_LUX_MAX_VALUE, configDataB2[IDX_LIGHT_LUX_MAX_VALUE]);
    }
    if (c == 'P') {
      n = ms.parseInt();
      configDataB2[IDX_PAINTED_H] = (n>>8) & 0xff;
      configDataB2[IDX_PAINTED_L] = (uint8_t)n & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_PAINTED_H, configDataB2[IDX_PAINTED_H]);
      EEPROM.write(IDX_PAINTED_L, configDataB2[IDX_PAINTED_L]);
    }
    if (c == 'U') {
      n = ms.parseInt();
      configDataB2[IDX_UNPAINTED_H] = (n>>8) & 0xff;
      configDataB2[IDX_UNPAINTED_L] = (uint8_t)n & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_UNPAINTED_H, configDataB2[IDX_UNPAINTED_H]);
      EEPROM.write(IDX_UNPAINTED_L, configDataB2[IDX_UNPAINTED_L]);
    }
  }
}
