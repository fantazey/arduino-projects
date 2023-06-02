#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DHT11_PIN PB1    // dht data pin
#define RX_PIN PB3
#define TX_PIN PB4
#else
#define DHT11_PIN 2       // dht data pin
#define RX_PIN 6       // software serial rx pin
#define TX_PIN 7       // software serial tx pin
#endif
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobot_DHT11.h>
#define DHT_I2C_ADDRESS 0x30

DFRobot_DHT11 DHT;
uint16_t dhtReadDelay = 5000;
uint32_t dhtTimer = 0;


SoftwareSerial ms(RX_PIN, TX_PIN);

#define CONFIG_DATA_IDX_CHANGED 0
#define CONFIG_DATA_IDX_MONTH 1
#define CONFIG_DATA_IDX_DAY 2
#define CONFIG_DATA_IDX_HOUR 3
#define CONFIG_DATA_IDX_MINUTE 4
#define CONFIG_DATA_IDX_SECOND 5
#define CONFIG_DATA_IDX_LIGHT_MODE 6
#define CONFIG_DATA_IDX_DHT_HUMIDITY 7
#define CONFIG_DATA_IDX_PAINTED_H 8
#define CONFIG_DATA_IDX_PAINTED_L 9
#define CONFIG_DATA_IDX_UNPAINTED_H 10
#define CONFIG_DATA_IDX_UNPAINTED_L 11
#define CONFIG_DATA_SIZE 32
// #define CONFIG_DATA_SIZE 12

uint8_t configData[CONFIG_DATA_SIZE] = {33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};
// uint8_t configData[CONFIG_DATA_SIZE] = {1, 5, 24, 23, 0, 56, 0, 9, 0, 25, 0, 200};
uint8_t dataIndex = 0;
uint8_t maxBlockSize = 8;

void setup() {
  ms.begin(9600);
  pinMode(DHT11_PIN, INPUT);
  Wire.begin(DHT_I2C_ADDRESS);
  Wire.onRequest(requestEvent);
  configData[CONFIG_DATA_IDX_PAINTED_H] = EEPROM.read(CONFIG_DATA_IDX_PAINTED_H);
  configData[CONFIG_DATA_IDX_PAINTED_L] = EEPROM.read(CONFIG_DATA_IDX_PAINTED_L);
  configData[CONFIG_DATA_IDX_UNPAINTED_H] = EEPROM.read(CONFIG_DATA_IDX_UNPAINTED_H);
  configData[CONFIG_DATA_IDX_UNPAINTED_L] = EEPROM.read(CONFIG_DATA_IDX_UNPAINTED_L);
  configData[CONFIG_DATA_IDX_LIGHT_MODE] = EEPROM.read(CONFIG_DATA_IDX_LIGHT_MODE);
}

void requestEvent() {
  uint8_t b = dataIndex;
  for (uint8_t i=b;i<b+maxBlockSize;i++) {
    // b1 = (configData[i]>>8) & 0xff;
    // b2 = configData[i] & 0xff;
    Wire.write(configData[i]);
    dataIndex++;
    if (dataIndex > CONFIG_DATA_SIZE) dataIndex = 0;
    // Wire.write(b2);
  }
  configData[CONFIG_DATA_IDX_CHANGED] = 0;
}

void loop() {
  if (millis() - dhtTimer > dhtReadDelay) {
    DHT.read(DHT11_PIN);
    configData[CONFIG_DATA_IDX_DHT_HUMIDITY] = DHT.humidity;
    dhtTimer = millis();
  }
  readDataUpdates();
}

void readDataUpdates() {  
  uint8_t c;
  while (ms.available() > 0) {
    c = ms.read();
    if (c == 'H') {
      configData[CONFIG_DATA_IDX_HOUR] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
    }
    if (c == 'M') {
      configData[CONFIG_DATA_IDX_MINUTE] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
    }
    if (c == 'S') {
      configData[CONFIG_DATA_IDX_SECOND] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
    }
    // if (c == 'y') {
    //   configData[CONFIG_DATA_IDX_YEAR] = ms.parseInt();
    //   configData[CONFIG_DATA_IDX_CHANGED] = 1;
    // }
    if (c == 'm') {
      configData[CONFIG_DATA_IDX_MONTH] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
    }
    if (c == 'd') {
      configData[CONFIG_DATA_IDX_DAY] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
    }
    if (c == 'L') {
      configData[CONFIG_DATA_IDX_LIGHT_MODE] = ms.parseInt();
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
      EEPROM.write(CONFIG_DATA_IDX_LIGHT_MODE, configData[CONFIG_DATA_IDX_LIGHT_MODE]);
    }
    if (c == 'P') {
      uint16_t n = ms.parseInt();
      configData[CONFIG_DATA_IDX_PAINTED_H] = (n>>8) & 0xff;
      configData[CONFIG_DATA_IDX_PAINTED_L] = (uint8_t)n & 0xff;
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
      EEPROM.write(CONFIG_DATA_IDX_PAINTED_H, configData[CONFIG_DATA_IDX_PAINTED_H]);
      EEPROM.write(CONFIG_DATA_IDX_PAINTED_L, configData[CONFIG_DATA_IDX_PAINTED_L]);
    }
    if (c == 'U') {
      uint16_t n = ms.parseInt();
      configData[CONFIG_DATA_IDX_UNPAINTED_H] = (n>>8) & 0xff;
      configData[CONFIG_DATA_IDX_UNPAINTED_L] = (uint8_t)n & 0xff;
      configData[CONFIG_DATA_IDX_CHANGED] = 1;
      EEPROM.write(CONFIG_DATA_IDX_UNPAINTED_H, configData[CONFIG_DATA_IDX_UNPAINTED_H]);
      EEPROM.write(CONFIG_DATA_IDX_UNPAINTED_L, configData[CONFIG_DATA_IDX_UNPAINTED_L]);
    }
  }
}
