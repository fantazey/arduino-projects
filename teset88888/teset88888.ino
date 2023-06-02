#include <Wire.h>
#include <SoftwareSerial.h>

#define TX_PIN 3
#define RX_PIN 4
SoftwareSerial mySerial(RX_PIN, TX_PIN);

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
#define CONFIG_DATA_SIZE 12

#define min(a, b) ((a) < (b) ? (a) : (b))
#define LUX_MAX_VAL_DYNAMIC_LIGHT = 80  // если выше - выключаем свет совсем
#define LUX_TO_BRIGHTNESS_MULT LUX_MAX_VAL_DYNAMIC_LIGHT / 255.0
#define LUX_TO_BRIGHTNESS_MULT 2.55


#define I2CADDRESS_RTC1307 0x50  // и еще какойто
#define I2CADDRESS_BMP280 0x76
#define I2CADDRESS_BH1750 0x23
#define I2CADDRESS_DISPLAY 0x29
#define I2CADDRESS_DHT11 0x30

// A4 - SDA - i2c data pin
// A5 - SCL - i2c clock pin

uint32_t configReadDelay = 10000;
uint32_t configTimer;
uint16_t dhtHumidity;


uint32_t lightSensorTimer;
uint16_t lightSensorTimerDelay = 10000;
float light;
uint8_t lightMode;

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;
bool needUpdateDigits = true;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  // инициализируем шину I2C
  Wire.begin();
  Serial.println("START");
}

void loop() {
  
  if (millis() - configTimer > configReadDelay) {    
    Serial.println("read config");
    readConfigModule();
    configTimer = millis();
    Serial.println("read 1");
  }
}

void readConfigModule() {
  Serial.println(F("Read config 1:"));
  uint8_t config[24];// = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t index = 0;
  uint8_t data = 0;
  Wire.requestFrom(I2CADDRESS_DHT11, 24);
  Serial.println(F("Read config:"));
  while (Wire.available() > 0 && index <= 24) {    
    data = Wire.read();
    Serial.print(F("index: "));
    Serial.println(index);
    Serial.print(F(" data: "));
    Serial.println(data);
    config[index++] = data;
  }
  bool updateTime = false;
  if (config[CONFIG_DATA_IDX_CHANGED] == 0) {
    dhtHumidity = config[CONFIG_DATA_IDX_DHT_HUMIDITY];
    return;
  }
  // RTC.read(tm);
  if (config[CONFIG_DATA_IDX_HOUR] != 0xffff) {
    // tm.Hour = config[CONFIG_DATA_IDX_HOUR];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_MINUTE] != 0xffff) {
    // tm.Minute = config[CONFIG_DATA_IDX_MINUTE];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_SECOND] != 0xffff) {
    // tm.Second = config[CONFIG_DATA_IDX_SECOND];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_MONTH] != 0xffff) {
    // tm.Month = config[CONFIG_DATA_IDX_MONTH];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_DAY] != 0xffff) {
    // tm.Day = config[CONFIG_DATA_IDX_DAY];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_LIGHT_MODE] != 0xffff) {
    lightMode = config[CONFIG_DATA_IDX_LIGHT_MODE];
  }
  if (config[CONFIG_DATA_IDX_PAINTED_L] != 0xffff) {
    paintedMiniatures = ((uint16_t)config[CONFIG_DATA_IDX_PAINTED_H] << 8) | config[CONFIG_DATA_IDX_PAINTED_L];
  }
  if (config[CONFIG_DATA_IDX_UNPAINTED_L] != 0xffff) {
    unpaintedMiniatures = ((uint16_t)config[CONFIG_DATA_IDX_UNPAINTED_H] << 8) | config[CONFIG_DATA_IDX_UNPAINTED_L];
  }
  if (!updateTime) {
    return;
  }
}
