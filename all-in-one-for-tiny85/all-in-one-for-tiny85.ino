#include <TimeLib.h>
#include <Wire.h>
#include <DFRobot_DHT11.h>
#include <DS1307RTC.h>
#include <Arduino.h>
#include <BMP280.h>

#define LED_CONTROL PB1  // output PWM led control
#define DHT11_PIN PB3       // dht data pin

#define LUX_TO_BRIGHTNESS_MULT 4.636
#define I2CADDRESS_RTC1307 0x50  // и еще какойто
#define I2CADDRESS_BMP280 0x76
#define I2CADDRESS_BH1750 0x23
#define I2CADDRESS_DISPLAY 0x29

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
#define MAXDISPLAYMODE 8
uint8_t activeDisplayMode;
uint32_t displayModeTimer;     // таймер текущего режима отображения
uint16_t displayModeDelay = 5000;  // 10sec

DFRobot_DHT11 DHT;
uint32_t dhtReadDelay = 10000;
uint32_t dhtTimer = 0;
float dhtHumidity;

uint32_t lightSensorTimer;
uint16_t lightSensorTimerDelay = 2000;
float light;

// 13416
BMP280 bmp280(I2CADDRESS_BMP280);
uint32_t bmpTimer;
uint16_t bmpTimerDelay = 10000;
float temperature;
float pressure;

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;
bool needUpdateDigits = true;
tmElements_t tm;

uint8_t brightness = 40;
uint8_t fadeAmount = 2;
uint8_t lightMode = 0;

void setup() {
  Serial.begin(9600);
  // инициализируем пин управления светом
  pinMode(LED_CONTROL, OUTPUT);
  // инициализируем шину I2C
  Wire.begin();
  // запускаем датчик интенсивности освещения с адресом по-умолчанию
  _BH1750begin();
  // запускаем датчик BMP280 с адресом 0x76
  bmp280.begin();
}

void loop() {
  // читаем серийный порт для обновления данных
  readDataUpdates();
  // измеряем температуру и влажность не слишком часто
  if (millis() - dhtTimer > dhtReadDelay) {
    readDHTSensor();
    dhtTimer = millis();
  }

  if (millis() - lightSensorTimer > lightSensorTimerDelay) {
    light = _BH1750light();
    lightSensorTimer = millis();
  }

  if (millis() - bmpTimer > bmpTimerDelay) {
    readBMP280Sensor();
    bmpTimer = millis();
  }

  // переключаем режимы экрана
  if (millis() - displayModeTimer > displayModeDelay) {
    displayModeTimer = millis();
    activeDisplayMode++;
    needUpdateDigits = true;
    if (activeDisplayMode > MAXDISPLAYMODE) {
      activeDisplayMode = 0;
    }
  }
  
  // выствляем яркость ангара
  lightUpHangar(lightMode);
  // рисуем на экране
  showData(activeDisplayMode);
}

void readDataUpdates() {  
  bool updateDate = false;
  uint8_t c;
  while (Serial.available() > 0) {
    RTC.read(tm);
    c = Serial.read();
    if (c == 'H') {
      tm.Hour = Serial.parseInt();
      updateDate = true;
    }
    if (c == 'M') {
      tm.Minute = Serial.parseInt();
      updateDate = true;
    }
    if (c == 'S') {
      tm.Second = Serial.parseInt();
      updateDate = true;
    }
    if (c == 'y') {
      tm.Year = CalendarYrToTm(Serial.parseInt());
      updateDate = true;
    }
    if (c == 'm') {
      tm.Month = Serial.parseInt();
      updateDate = true;
    }
    if (c == 'd') {
      tm.Day = Serial.parseInt();
      updateDate = true;
    }
    if (c == 'L') {
      lightMode = Serial.parseInt();
    }
    if (c == 'P') {
      paintedMiniatures = Serial.parseInt();
    }
    if (c == 'U') {
      unpaintedMiniatures = Serial.parseInt();
    }
    if (c == '\n') {
      // Serial.println("end read data");
      ;
    }
  }
  if (updateDate) {
    RTC.write(tm);
  }
}

void readDHTSensor() {
  DHT.read(DHT11_PIN);
  dhtHumidity = DHT.humidity;
}

void readBMP280Sensor() {  
  temperature = bmp280.getTemperature();
  pressure = bmp280.getPressure() * 0.01 * 0.75006156;
}

void showData(uint8_t mode) {
  tmElements_t readTime;
  if (!needUpdateDigits) {
    drawDigits();
    return;
  }
  switch (mode) {
    case 0:
      RTC.read(readTime);
      setNumber(readTime.Day * 100 + readTime.Month);
      break;
    case 1:
      RTC.read(readTime);
      setNumber(readTime.Hour * 100 + readTime.Minute);
      break;
    case 2:
      setNumberF(temperature, 2);
      break;
    case 3:
      setNumberF(pressure, 1);
      break;
    case 4:      
      setNumberF(dhtHumidity, 2);
      break;    
    case 5:
      setNumberF(light, 1);
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
      brightness = 255 - (light * LUX_TO_BRIGHTNESS_MULT);
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

/**
 * Вывести десятичное число с нужным количеством десятичных знаков (помним что экран только 4 символа)
 */
void setNumberF(float numToShow, uint8_t dec) {
  floatToBuff(numToShow, dec);
  decPlaces = dec;
}

void intToBuff(uint16_t number) {
  uint8_t index = 0;
  uint16_t rest = number;
  int8_t d = 0;
  do {
    d = rest % 10;
    rest = rest / 10;
    numberBuffer[index++] = d;
  } while (rest > 0);
}

void floatToBuff(float number, uint8_t decPlaces) {
  int16_t powersOf10[] = {
    1,      // 10^0
    10,     // 10^1
    100,    // 10^2
    1000,   // 10^3
    10000,  // 10^4
  };
  uint16_t intNumber = number * powersOf10[decPlaces];
  intToBuff(number);
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
    Wire.beginTransmission(I2CADDRESS_BH1750);
    Wire.write(0x10); // Continuously H-Resolution Mode
    if (Wire.endTransmission() == 0) {
        delay(180); // Wait to complete 1st H-resolution mode measurement. (max. 180ms.) 
        return true;
    }
    return false;
}


float _BH1750light() {
    int n = Wire.requestFrom(I2CADDRESS_BH1750, 2);
    if (n != 2) {
        return -1.0;
    }
    uint16_t val = 0;
    val = Wire.read();
    val <<= 8;
    val |= Wire.read();
    return val / 1.2;    
}