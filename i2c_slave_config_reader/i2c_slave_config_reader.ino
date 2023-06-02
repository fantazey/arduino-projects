#include <Wire.h>
#include <SoftwareSerial.h>

#define CONFIG_I2C_ADDRESS 0x33

#define RX PB3
#define TX PB4
SoftwareSerial ms(RX, TX);

#define CONFIG_DATA_IDX_YEAR 0
#define CONFIG_DATA_IDX_MONTH 1
#define CONFIG_DATA_IDX_DAY 2
#define CONFIG_DATA_IDX_HOUR 3
#define CONFIG_DATA_IDX_MINUTE 4
#define CONFIG_DATA_IDX_SECOND 5
#define CONFIG_DATA_IDX_LIGHT_MODE 6
#define CONFIG_DATA_IDX_PAINTED 7
#define CONFIG_DATA_IDX_UNPAINTED 8
#define CONFIG_DATA_SIZE 9

uint16_t configData[CONFIG_DATA_SIZE];  //ymdHMSLPU
bool dataUpdated = false;

void setup() {
  Serial.begin(9600);
  ms.begin(9600);
  Wire.begin(CONFIG_I2C_ADDRESS);

}

void loop() {
  readDataUpdates();
}

void readDataUpdates() {  
  uint8_t c;
  while (ms.available() > 0) {
    c = ms.read();
    if (c == 'H') {
      configData[CONFIG_DATA_IDX_HOUR] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'M') {
      configData[CONFIG_DATA_IDX_MINUTE] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'S') {
      configData[CONFIG_DATA_IDX_SECOND] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'y') {
      configData[CONFIG_DATA_IDX_YEAR] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'm') {
      configData[CONFIG_DATA_IDX_MONTH] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'd') {
      configData[CONFIG_DATA_IDX_DAY] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'L') {
      configData[CONFIG_DATA_IDX_LIGHT_MODE] = ms.parseInt();
      dataUpdated = true;
    }
    if (c == 'P') {
      configData[CONFIG_DATA_IDX_PAINTED] = ms.parseInt();
      dataUpdated = true;  
    }
    if (c == 'U') {
      configData[CONFIG_DATA_IDX_UNPAINTED] = ms.parseInt();
      dataUpdated = true;  
    }
  }
}
