#include <EEPROM.h>
// #include <Wire.h>
#include <SoftwareSerial.h>
#include <DFRobot_DHT11.h>

#define DHT_I2C_ADDRESS 0x30

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DHT11_PIN PB1  // dht data pin
#define RX_PIN PB3
#define TX_PIN PB4

#define SERIAL_IN PB0
#define COMMAND_IN PB1
#define DATA_IN PB2

#define debug_serial digitalWrite(SERIAL_IN, HIGH)
#define debug_command digitalWrite(COMMAND_IN, HIGH)
#define debug_data digitalWrite(DATA_IN, HIGH)

#else
#define DHT11_PIN 2  // dht data pin
#define RX_PIN 3     // software serial rx pin
#define TX_PIN 4     // software serial tx pin
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
#define BLOCK_SIZE 10
uint8_t configDataB1[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t configDataB2[BLOCK_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8_t blockToSend = 1;

DFRobot_DHT11 DHT;
const uint16_t _10sec = 10000;
uint32_t dhtTimer = 0;
uint8_t size = 0;

SoftwareSerial ms(RX_PIN, TX_PIN);

uint16_t parsedNumber;

void setup() {
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  ms.begin(9600);
  pinMode(SERIAL_IN, OUTPUT);
  pinMode(COMMAND_IN, OUTPUT);
  pinMode(DATA_IN, OUTPUT);
  // pinMode(DHT11_PIN, INPUT);
  // Wire.begin(DHT_I2C_ADDRESS);
  // Wire.onRequest(requestEvent);
  // Wire.onReceive(receiveEvent);
  configDataB2[IDX_PAINTED_H] = EEPROM.read(IDX_PAINTED_H);
  configDataB2[IDX_PAINTED_L] = EEPROM.read(IDX_PAINTED_L);
  configDataB2[IDX_UNPAINTED_H] = EEPROM.read(IDX_UNPAINTED_H);
  configDataB2[IDX_UNPAINTED_L] = EEPROM.read(IDX_UNPAINTED_L);
  configDataB2[IDX_LIGHT_MODE] = EEPROM.read(IDX_LIGHT_MODE);
  configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = EEPROM.read(IDX_LIGHT_LUX_MAX_VALUE);
  configDataB2[IDX_DISPLAY_MODE] = EEPROM.read(IDX_DISPLAY_MODE);
}

// void requestEvent() {
//   if (blockToSend == 1) {
//     Wire.write(250); // control byte begin of data
//     for (uint8_t i = 0; i < BLOCK_SIZE; i++) {
//       Wire.write(configDataB1[i]);
//     }
//     Wire.write(251); // control byte end of data
//     configDataB1[IDX_CHANGED] = 0;
//   } else {
//     Wire.write(252); // control byte begin of data
//     for (uint8_t i = 0;i < BLOCK_SIZE; i++) {
//       Wire.write(configDataB2[i]);
//     }
//     Wire.write(253); // control byte end of data
//     configDataB2[IDX_CHANGED] = 0;
//   }
// }

// void receiveEvent() {
//   uint8_t c;
//   if (Wire.available() > 0) {
//     c = Wire.read();
//     if (c == 250) {
//       blockToSend = Wire.read();
//     }
//   }
// }

void loop() {
  if (millis() - dhtTimer > _10sec) {
    // DHT.read(DHT11_PIN);
    configDataB1[IDX_DHT_HUMIDITY] = 12;     //DHT.humidity;
    configDataB1[IDX_DHT_TEMPERATURE] = 16;  //DHT.temperature;
    digitalWrite(SERIAL_IN, LOW);
    digitalWrite(COMMAND_IN, LOW);
    digitalWrite(DATA_IN, LOW);
    dhtTimer = millis();
  }
  readDataUpdates();
}
// todo: вытащить буффер в глобальные переменные
void readDataUpdates() {
  uint8_t c;
  bool startCommandReceived = false;
  bool allDataReceived = false;
  uint8_t index = 0;
  uint8_t index2 = 0;
  uint8_t buff[64];
  while (ms.available() > 0) {
    debug_serial;
    debug_data;
    c = ms.read();
    if (c == '#') {
      
      allDataReceived = true;
      continue;
    }
    if (c == '@') {
      debug_command;
      size = 0;
      startCommandReceived = true;
      continue;
    }
    if (startCommandReceived)
      buff[size++] = c;
  }
  if (size == 0 || !allDataReceived) {
    return;
  }

  while (index < size) {
    c = buff[index];
    index++;
    if (c == 'H') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_HOUR] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    if (c == 'M') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_MINUTE] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    if (c == 'S') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_SECOND] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    if (c == 'y') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_YEAR] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    if (c == 'm') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_MONTH] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    if (c == 'd') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB1[IDX_DAY] = (uint8_t)parsedNumber;
      configDataB1[IDX_CHANGED] = 1;
    }

    //==========================================================
    if (c == 'A') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB2[IDX_DISPLAY_MODE] = (uint8_t)parsedNumber;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_DISPLAY_MODE, configDataB2[IDX_DISPLAY_MODE]);
    }

    if (c == 'L') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB2[IDX_LIGHT_MODE] = (uint8_t)parsedNumber;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_LIGHT_MODE, configDataB2[IDX_LIGHT_MODE]);
    }

    if (c == 'l') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB2[IDX_LIGHT_LUX_MAX_VALUE] = (uint8_t)parsedNumber;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_LIGHT_LUX_MAX_VALUE, configDataB2[IDX_LIGHT_LUX_MAX_VALUE]);
    }

    if (c == 'P') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB2[IDX_PAINTED_H] = (parsedNumber >> 8) & 0xff;
      configDataB2[IDX_PAINTED_L] = (uint8_t)parsedNumber & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_PAINTED_H, configDataB2[IDX_PAINTED_H]);
      EEPROM.write(IDX_PAINTED_L, configDataB2[IDX_PAINTED_L]);
    }

    if (c == 'U') {
      index2 = readIntFromBuff(buff, index, size);
      if (index == index2) {
        continue;
      }
      index = index2;
      configDataB2[IDX_UNPAINTED_H] = (parsedNumber >> 8) & 0xff;
      configDataB2[IDX_UNPAINTED_L] = (uint8_t)parsedNumber & 0xff;
      configDataB2[IDX_CHANGED] = 1;
      EEPROM.write(IDX_UNPAINTED_H, configDataB2[IDX_UNPAINTED_H]);
      EEPROM.write(IDX_UNPAINTED_L, configDataB2[IDX_UNPAINTED_L]);
    }

    if (c == 'G') {
      ms.println("b1:");
      for (uint8_t k = 0; k < BLOCK_SIZE; k++) {
        ms.print("index: ");
        ms.print(k);
        ms.print(" data: ");
        ms.println(configDataB1[k]);
      }
      ms.println("Block 2:");
      for (uint8_t k = 0; k < BLOCK_SIZE; k++) {
        ms.print("index: ");
        ms.print(k);
        ms.print(" data: ");
        ms.println(configDataB2[k]);
      }
    }
  }
  allDataReceived = false;
}

// P12U343#
// P54#
// return last index
uint8_t readIntFromBuff(int8_t* buff, uint8_t startIndex, uint8_t size) {
  parsedNumber = 0;
  uint32_t result = 0;
  uint8_t digitsBuff[6] = { 0, 0, 0, 0, 0, 0 };
  uint8_t count = 0;
  uint8_t index = startIndex;
  while (index < size) {
    if (buff[index] < 48 || buff[index] > 57) {
      break;
    }
    digitsBuff[count++] = buff[index] - 48;
    index++;
  }
  if (count == 0) {
    return index;
  }

  for (uint8_t i = 0; i < count; i++) {
    result += digitsBuff[i];
    result *= 10;
  }
  parsedNumber = result / 10;
  return index;
}

