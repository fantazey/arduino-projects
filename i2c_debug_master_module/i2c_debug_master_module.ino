#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>
// #include <BMP280.h>
#include <SoftwareSerial.h>

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define LED_CONTROL PB1  // output PWM led control
#define TX_PIN PB3
#define RX_PIN PB4
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#else
#define LED_CONTROL 9  // output PWM led control
#define TX_PIN 3
#define RX_PIN 4
SoftwareSerial mySerial(RX_PIN, TX_PIN);

#endif

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

uint8_t numberBuffer[4];
uint8_t decPlaces;

/**
 * 0 - показываем дату 
 * 1 - показываем время     
 * 2 - показываем температуру bmp
 * 3 - показываем давление bmp
 * 4 - показываем влажность dht
 * 5 - показываем интенсивность света  
 * 6 - показываем сколько покрашено
 * 7 - показываем сколько не покрашено
 */
#define MAXDISPLAYMODE 7
uint8_t activeDisplayMode;
uint32_t displayModeTimer;          // таймер текущего режима отображения
uint16_t displayModeDelay = 10000;  // 10sec

uint32_t configReadDelay = 10000;
uint32_t configTimer;
uint16_t dhtHumidity;


uint32_t lightSensorTimer;
uint16_t lightSensorTimerDelay = 10000;
float light;

// 13416
// BMP280 bmp280(I2CADDRESS_BMP280);
uint32_t bmpTimer;
uint16_t bmpTimerDelay = 10000;
float temperature;
float pressure;

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;
bool needUpdateDigits = true;

uint32_t readTimeTimer;
uint16_t readTimeDelay = 5000;
tmElements_t tm;

uint8_t brightness = 40;
uint8_t fadeAmount = 2;
uint8_t lightMode = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("START1");
  mySerial.begin(9600);
  Serial.println("START2");
  // инициализируем пин управления светом
  pinMode(LED_CONTROL, OUTPUT);
  Serial.println("START3");
  // инициализируем шину I2C
  Wire.begin();
  Serial.println("START4");
  // запускаем датчик интенсивности освещения с адресом по-умолчанию
  _BH1750begin();
  Serial.println("START5");
  // запускаем датчик BMP280 с адресом 0x76
  // bmp280.begin();
  Serial.println("START6");
  Serial.println("START");
}

void loop() {
  Serial.println("loop");
  // измеряем температуру и влажность и настройки не слишком часто
  if (millis() - configTimer > configReadDelay) {    
    Serial.println("read config");
    readConfigModule();
    configTimer = millis();
    Serial.println("read 1");
  }
  if (millis() - lightSensorTimer > lightSensorTimerDelay) {    
    Serial.println("read light");
    light = _BH1750light();
    lightSensorTimer = millis();
    Serial.println("read 2");
  }
  Serial.println("before read serial");
  if (mySerial.available() > 0) {
    Serial.println(F("data from config:"));
    while (mySerial.available() > 0) {
      Serial.print(mySerial.read());
    }
    Serial.println(F("\nend data from config"));
  }
  
  Serial.println("after read serial");
  if (millis() - bmpTimer > bmpTimerDelay) {    
    Serial.println("read bmp");
    // readBMP280Sensor();
    bmpTimer = millis();
    Serial.println("read 3");
  }
  if (millis() - readTimeTimer > readTimeDelay) {    
    Serial.println("read time");
    RTC.read(tm);
    readTimeTimer = millis();
    Serial.println("read 4");
  }
  Serial.println("before show data");
  // рисуем на экране
  // showData(activeDisplayMode);
  Serial.println("after show data");
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
  RTC.read(tm);
  if (config[CONFIG_DATA_IDX_HOUR] != 0xffff) {
    tm.Hour = config[CONFIG_DATA_IDX_HOUR];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_MINUTE] != 0xffff) {
    tm.Minute = config[CONFIG_DATA_IDX_MINUTE];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_SECOND] != 0xffff) {
    tm.Second = config[CONFIG_DATA_IDX_SECOND];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_MONTH] != 0xffff) {
    tm.Month = config[CONFIG_DATA_IDX_MONTH];
    updateTime = true;
  }
  if (config[CONFIG_DATA_IDX_DAY] != 0xffff) {
    tm.Day = config[CONFIG_DATA_IDX_DAY];
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
  tm.Year = CalendarYrToTm(2023);
  tmElements_t tm2;
  RTC.read(tm2);
  if (makeTime(tm2) < makeTime(tm)) {
    RTC.write(tm);
  }
}

void readBMP280Sensor() {
  // temperature = bmp280.getTemperature();
  // pressure = bmp280.getPressure() * 0.01 * 0.75006156;
}

void showData(uint8_t mode) {
  if (!needUpdateDigits) {
    return;
  }
  uint16_t intData = 0;
  switch (mode) {
    case 0:
      setNumber(tm.Day * 100 + tm.Month);
      break;
    case 1:
      setNumber(tm.Hour * 100 + tm.Minute);
      break;
    case 2:
      setNumber(1234);
      break;
    case 3:
      setNumber(2345);
      break;
    case 4:
      setNumber(3456);
      break;
    case 5:
      setNumber(4567);
      break;
    case 6:
      setNumber(paintedMiniatures);
      break;
    case 7:
      setNumber(unpaintedMiniatures);
      break;
  }
  needUpdateDigits = false;
  drawDigits();
}


void lightUpHangar(uint8_t mode) {
  switch (mode) {
    case 0:
      // off light
      brightness = 0;
      break;
    case 1:
      // dynamic light
      brightness = 255 - min(255, (light * LUX_TO_BRIGHTNESS_MULT));
      break;
    case 2:
      brightness = brightness + fadeAmount;
      if (brightness <= 5 || brightness >= 100) {
        fadeAmount = -fadeAmount;
      }
      break;
    case 3:
      // low light
      brightness = 10;
      break;
    case 4:
      // mid light
      brightness = 100;
      break;
    case 5:
      // bright light
      brightness = 255;
      break;
  }
  analogWrite(LED_CONTROL, brightness);
}


/**
 * Вывести целое число
 */
void setNumber(int16_t numToShow) {
  intToBuff(numToShow);
}


void intToBuff(uint16_t number) {
  numberBuffer[0] = 0;
  numberBuffer[1] = 0;
  numberBuffer[2] = 0;
  numberBuffer[3] = 0;
  decPlaces = 0;
  uint8_t index = 0;
  uint16_t rest = number;
  int8_t d = 0;
  do {
    d = rest % 10;
    rest = rest / 10;
    numberBuffer[index++] = d;
  } while (rest > 0);
}

void drawDigits() {
  Wire.beginTransmission(I2CADDRESS_DISPLAY);
  Wire.write(0xff);
  for (uint8_t i = 0; i < 4; i++) {
    Wire.write(numberBuffer[i]);
  }
  Wire.write(0xfe);
  Wire.write(0xfd);
  Wire.write(decPlaces);
  Wire.write(0xfc);
  Wire.endTransmission();
}


void _BH1750begin() {
  // Wire.beginTransmission(I2CADDRESS_BH1750);
  // Wire.write(0x10);  // Continuously H-Resolution Mode
  // if (Wire.endTransmission() == 0) {
  //   delay(180);  // Wait to complete 1st H-resolution mode measurement. (max. 180ms.)
  //   return true;
  // }
  // return false;
}


float _BH1750light() {
  // int n = Wire.requestFrom(I2CADDRESS_BH1750, 2);
  // if (n != 2) {
  //   return -1.0;
  // }
  // uint16_t val = 0;
  // val = Wire.read();
  // val <<= 8;
  // val |= Wire.read();
  // return val / 1.2;
}