#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobot_DHT11.h>

#define DHT_I2C_ADDRESS 0x30

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DHT11_PIN PB1    // dht data pin
#define RX_PIN PB3
#define TX_PIN PB4
#define SERIAL_IN_PIN PB1
#define COMMAND_IN_PIN PB2
#define DATA_IN_PIN PB0
#else
#define DHT11_PIN 2       // dht data pin
#define RX_PIN 3       // software serial rx pin
#define TX_PIN 4       // software serial tx pin
#endif

#define to16(h, l) (((uint16_t)h << 8) | l)

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
#define BLOCK_SIZE 11
uint8_t configDataB1[BLOCK_SIZE] = {0,0,0,0,0, 0,0,0,0,0, 251};
uint8_t configDataB2[BLOCK_SIZE] = {1,23,0,15,10, 20,30,40,50,60, 253};

uint8_t blockToSend = 1;

// DFRobot_DHT11 DHT;
const uint16_t _10sec = 10000;
uint32_t dhtTimer = 0;

SoftwareSerial ms(RX_PIN, TX_PIN);

void setup() {  
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  ms.begin(9600);
  // pinMode(DHT11_PIN, INPUT);
  pinMode(SERIAL_IN_PIN, OUTPUT);
  pinMode(COMMAND_IN_PIN, OUTPUT);
  pinMode(DATA_IN_PIN, OUTPUT);
  // digitalWrite(DHT11_PIN, HIGH);
  // Wire.begin(DHT_I2C_ADDRESS);
  // Wire.onRequest(requestEvent);
  // Wire.onReceive(receiveEvent);
  // ms.listen();
  // configDataB2[IDX_PAINTED_H] = EEPROM.read(IDX_PAINTED_H);
  // configDataB2[IDX_PAINTED_L] = EEPROM.read(IDX_PAINTED_L);
  // configDataB2[IDX_UNPAINTED_H] = EEPROM.read(IDX_UNPAINTED_H);
  // configDataB2[IDX_UNPAINTED_L] = EEPROM.read(IDX_UNPAINTED_L);
  // configDataB2[IDX_LIGHT_MODE] = EEPROM.read(IDX_LIGHT_MODE);
  // configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = EEPROM.read(IDX_LIGHT_LUX_MAX_VALUE);
  // configDataB2[IDX_DISPLAY_MODE] = EEPROM.read(IDX_DISPLAY_MODE);  
}

void requestEvent() {
  if (blockToSend == 1) {
    Wire.write(250); // control byte begin of data
    for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
      Wire.write(configDataB1[i]);
    }
    // Wire.write(251); // control byte end of data
    configDataB1[IDX_CHANGED] = 0;
  } else {
    Wire.write(252); // control byte begin of data
    for (uint8_t i = 0;i < BLOCK_SIZE; i++) {
      Wire.write(configDataB2[i]);
    }
    // Wire.write(253); // control byte end of data
    configDataB2[IDX_CHANGED] = 0;
  }
  // uint8_t* configBlock = blockToSend == 1 ? configDataB1 : configDataB2;
  // configBlock[IDX_CHANGED] = 0;
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
  if (millis() - dhtTimer > _10sec) {
    // DHT.read(DHT11_PIN);
    // configDataB1[IDX_DHT_HUMIDITY] = DHT.humidity;
    // configDataB1[IDX_DHT_TEMPERATURE] = DHT.temperature;
    dhtTimer = millis();
    digitalWrite(SERIAL_IN_PIN, LOW);
    digitalWrite(DATA_IN_PIN, LOW);
    digitalWrite(COMMAND_IN_PIN, LOW);
  }
  readDataUpdates();
}

void readDataUpdates() {  
  uint8_t c;
  uint16_t n = 0;
  bool commandBeginReceived = false;
  while (ms.available() > 0) {            
    c = ms.read();
    digitalWrite(SERIAL_IN_PIN, HIGH);
    if (c == '@') {      
      commandBeginReceived = true;
      digitalWrite(COMMAND_IN_PIN, HIGH);
      // continue;
    }
    // if (!commandBeginReceived) {
    //   continue;
    // }
    if (c == 'H') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_HOUR] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'M') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_MINUTE] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'S') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_SECOND] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'y') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_YEAR] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'm') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_MONTH] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    if (c == 'd') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB1[IDX_DAY] = ms.parseInt();
      configDataB1[IDX_CHANGED] = 1;
    }
    //==========================================================
    if (c == 'A') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB2[IDX_DISPLAY_MODE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      // EEPROM.write(IDX_DISPLAY_MODE, configDataB2[IDX_DISPLAY_MODE]);      
    }
    if (c == 'L') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB2[IDX_LIGHT_MODE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      // EEPROM.write(IDX_LIGHT_MODE, configDataB2[IDX_LIGHT_MODE]);
    }
    if (c == 'l') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = ms.parseInt();
      configDataB2[IDX_CHANGED] = 1;
      // EEPROM.write(IDX_LIGHT_LUX_MAX_VALUE, configDataB2[IDX_LIGHT_LUX_MAX_VALUE]);
    }
    if (c == 'P') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      n = ms.parseInt();
      configDataB2[IDX_PAINTED_H] = (n>>8) & 0xff;
      configDataB2[IDX_PAINTED_L] = (uint8_t)n & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      digitalWrite(DATA_IN_PIN, HIGH);
      // EEPROM.write(IDX_PAINTED_H, configDataB2[IDX_PAINTED_H]);
      // EEPROM.write(IDX_PAINTED_L, configDataB2[IDX_PAINTED_L]);      
    }
    if (c == 'U') {
      digitalWrite(COMMAND_IN_PIN, HIGH);
      n = ms.parseInt();
      configDataB2[IDX_UNPAINTED_H] = (n>>8) & 0xff;
      configDataB2[IDX_UNPAINTED_L] = (uint8_t)n & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      digitalWrite(DATA_IN_PIN, LOW);
      // EEPROM.write(IDX_UNPAINTED_H, configDataB2[IDX_UNPAINTED_H]);
      // EEPROM.write(IDX_UNPAINTED_L, configDataB2[IDX_UNPAINTED_L]);
    }
  }
}
