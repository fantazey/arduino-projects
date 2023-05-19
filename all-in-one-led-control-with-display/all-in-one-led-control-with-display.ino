#include <TimeLib.h>
#include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <DHT.h>
// #include <DHT_U.h>
#include <DS1307RTC.h>
#include <Arduino.h>
#include <ArtronShop_BH1750.h>
#include <Adafruit_BMP280.h>

#define LED_CONTROL 9  // output PWM led control
// #define DHTPIN 2       // dht data pin
// #define DHTTYPE DHT11  // DHT 11
#define LATCH_PIN 4  // display RCLK pin connected
#define CLOCK_PIN 3  // display SCLK pin connected
#define DATA_PIN 5   // display DIO pin connected


#define LUX_TO_BRIGHTNESS_MULT 4.636
#define I2CADDRESS_RTC1307 0
#define I2CADDRESS_BMP280 0x76
#define I2CADDRESS_BH1750 0x23

// A4 - SDA - i2c data pin
// A5 - SCL - i2c clock pin
// битмапа зажигания символов на семисегментном экране
static const uint8_t digitCodeMap[] = {
  // GFEDCBA  Segments      7-segment map:
  0b00000011,  // 0   "0"          AAA
  0b10011111,  // 1   "1"         F   B
  0b00100101,  // 2   "2"         F   B
  0b00001101,  // 3   "3"          GGG
  0b10011001,  // 4   "4"         E   C
  0b01001001,  // 5   "5"         E   C
  0b01000001,  // 6   "6"          DDD
  0b00011111,  // 7   "7"
  0b00000001,  // 8   "8"
  0b00001001,  // 9   "9"
  0b11111111,  // 32  ' '  BLANK
  0b11111101,  // 45  '-'  DASH
  0b11111110,  // 46  '.'  PERIOD
  0b11101111,  // 95 '_'  UNDERSCORE
};

#define BLANK_IDX 10       // "пробел" индекс должен совпадать с битмапой в 'digitCodeMap'
#define DASH_IDX 11        // "дефис" индекс должен совпадать с битмапой в 'digitCodeMap'
#define PERIOD_IDX 12      // "точка" индекс должен совпадать с битмапой в 'digitCodeMap'
#define UNDERSCORE_IDX 13  // "подчеркивание" индекс должен совпадать с битмапой в 'digitCodeMap'
#define MAXNUMDIGITS 4     // максимальное число знаков

int8_t numDigits = MAXNUMDIGITS;   // число знаков на экране
uint8_t digitCodes[MAXNUMDIGITS];  // хранение битмап для каждой позиции для отображения на экране
static const int16_t powersOf10[] = {
  1,      // 10^0
  10,     // 10^1
  100,    // 10^2
  1000,   // 10^3
  10000,  // 10^4
};

/**
 * 0 - показываем дату 
 * 1 - показываем время    
 * 2 - показываем температуру dht
 * 3 - показываем влажность dht
 * 4 - показываем температуру bmp
 * 5 - показываем давление bmp
 * 6 - показываем интенсивность света  
 * 7 - показываем сколько покрашено
 * 8 - показываем сколько не покрашено
 */
#define MAXDISPLAYMODE 8
uint8_t activeDisplayMode = 0;
uint32_t displayModeTimer = 0;     // таймер текущего режима отображения
uint16_t displayModeDelay = 5000;  // 10sec

// DHT_Unified dht(DHTPIN, DHTTYPE);
// uint32_t dhtReadDelay = 10000;
// uint32_t dhtTimer = 0;
float dhtTemperature = 0;
float dhtHumidity = 0;


ArtronShop_BH1750 bh1750(0x23, &Wire); // Non Jump ADDR: 0x23, Jump ADDR: 0x5C
uint32_t lightSensorTimer = 0;
uint16_t lightSensorTimerDelay = 2000;
float light = 0;


Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
uint32_t bmpTimer = 0;
uint16_t bmpTimerDelay = 10000;
float temperature = 0;
float pressure = 0;

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;
bool needUpdateDigits = true;
tmElements_t tm;

uint8_t lightMode = 0;

void setup() {
  Serial.begin(9600);
  // инициализируем пин управления светом
  pinMode(LED_CONTROL, OUTPUT);
  // инициализируем пины экрана
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  // инициализируем работу с датчиком dht11
  // dht.begin();
  // sensor_t sensor;
  // dht.temperature().getSensor(&sensor);
  // dhtReadDelay = sensor.min_delay / 100;
  // инициализируем шину I2C
  Wire.begin();
  // запускаем датчик интенсивности освещения с адресом по-умолчанию
  bh1750.begin();
  // запускаем датчик BMP280 с адресом 0x76
  bmp.begin(0x76);
  // настраиваем датчик BMP280
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  // инициализируем RTC модуль
  tm.Hour = 12;
  tm.Minute = 0;
  tm.Second = 45;
  tm.Day = 15;
  tm.Month = 4;
  tm.Year = CalendarYrToTm(2023);
  RTC.write(tm);
  RTC.read(tm);
  // Serial.println("setup ok");
}

void loop() {
  // читаем серийный порт для обновления данных
  readDataUpdates();

  // измеряем температуру и влажность не слишком часто
  // if (millis() - dhtTimer > dhtReadDelay) {
  //   readDHTSensor();
  //   dhtTimer = millis();
  // }

  if (millis() - lightSensorTimer > lightSensorTimerDelay) {
    readLightSensor();
    lightSensorTimer = millis();
  }

  if (millis() - bmpTimer > bmpTimerDelay) {
    readBMP280Sensor();
    bmpTimer = millis();
  }

  // переключаем режимы экрана
  if (millis() - displayModeTimer > displayModeDelay) {
    displayModeTimer = millis();
    activeDisplayMode++;// = activeDisplayMode + (uint8_t)1;
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
  tmElements_t outerTime;
  RTC.read(outerTime);
  while (Serial.available() > 0) {
    uint8_t c = Serial.read();
    if (c == 'H') {
      outerTime.Hour = Serial.parseInt();
      RTC.write(outerTime);
    }
    if (c == 'M') {
      outerTime.Minute = Serial.parseInt();
      RTC.write(outerTime);
    }
    if (c == 'S') {
      outerTime.Second = Serial.parseInt();
      RTC.write(outerTime);
    }
    // if (c == 'y') {
    //   outerTime.Year = CalendarYrToTm(Serial.parseInt());
    //   RTC.write(outerTime);
    // }
    if (c == 'm') {
      outerTime.Month = Serial.parseInt();
      RTC.write(outerTime);
    }
    if (c == 'd') {
      outerTime.Day = Serial.parseInt();
      RTC.write(outerTime);
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
}

// void readDHTSensor() {
//   sensors_event_t event;
//   dht.temperature().getEvent(&event);
//   if (isnan(event.temperature)) {
//     dhtTemperature = 0.0;
//   } else {
//     dhtTemperature = event.temperature;
//   }
//   dht.humidity().getEvent(&event);
//   if (isnan(event.relative_humidity)) {
//     dhtHumidity = 0.0;
//   } else {
//     dhtHumidity = event.relative_humidity;
//   }
// }

void readLightSensor() {
  light = bh1750.light();
}

void readBMP280Sensor() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  temperature = temp_event.temperature;
  pressure = pressure_event.pressure;
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
      // setNumberF(readTime.Month + ((float)readTime.Day / 100.0), 2);
      setNumber(readTime.Day * 100 + readTime.Month);
      break;
    case 1:
      RTC.read(readTime);
      setNumberF(readTime.Hour + ((float)readTime.Minute / 100.0), 2);
      // setNumber(readTime.Hour * 100 + readTime.Minute);
      break;
    case 2:
      setNumberF(dhtTemperature, 2);
      break;
    case 3:
      setNumberF(dhtHumidity, 2);      
      break;
    case 4:
      setNumberF(temperature, 2);
      break;
    case 5:
      setNumberF(pressure * 0.75006156, 1);
      break;
    case 6:
      setNumberF(light, 1);
      break;
    case 7:
      setNumber(paintedMiniatures);      
      break;
    case 8:
      setNumber(unpaintedMiniatures);
      break;
  }
  needUpdateDigits = false;
  drawDigits();
}


void lightUpHangar(uint8_t mode) {
  switch(mode) {
    case 0:
      reset();
      break;
    case 1:
      analogWrite(LED_CONTROL, (uint8_t)(255 - (light * LUX_TO_BRIGHTNESS_MULT)));
      break;
    case 2:
      lowGlow();
      break;
    case 3:
      midGlow();
      break;
    case 4:
      brightGlow();
      break;
  }
}

void lowGlow() {
  analogWrite(LED_CONTROL, 10);
}

void midGlow() {
  analogWrite(LED_CONTROL, 100);
}

void brightGlow() {
  analogWrite(LED_CONTROL, 255);
}

void reset() {
    analogWrite(LED_CONTROL, 0);
}


/**
 * Вывести целое число
 */
void setNumber(int16_t numToShow) {
  setNewNum(numToShow, -1);
}

/**
 * Вывести десятичное число с нужным количеством десятичных знаков (помним что экран только 4 символа)
 */
void setNumberF(float numToShow, int8_t decPlaces) {  //float
  int8_t decPlacesPos = constrain(decPlaces, 0, MAXNUMDIGITS);
  numToShow = numToShow * powersOf10[decPlacesPos];
  // Modify the number so that it is rounded to an integer correctly
  numToShow += (numToShow >= 0.f) ? 0.5f : -0.5f;
  setNewNum((int16_t)numToShow, (int8_t)decPlaces);
}

/**
 * Вывести число с нужным количеством десятичных знаков на экране
 */
void setNewNum(int16_t numToShow, int8_t decPlaces) {
  uint8_t digits[MAXNUMDIGITS];
  findDigits(numToShow, decPlaces, digits);
  setDigitCodes(digits, decPlaces);
}

/**
 * Конвертация числа в массив знаков который (спиздил в какой то библиотеке)
 */
void findDigits(int16_t numToShow, int8_t decPlaces, uint8_t digits[]) {
  const int16_t* powersOfBase = powersOf10;
  const int16_t maxNum = powersOfBase[numDigits] - 1;
  const int16_t minNum = -(powersOfBase[numDigits - 1] - 1);

  // If the number is out of range, just display dashes
  if (numToShow > maxNum || numToShow < minNum) {
    for (uint8_t digitNum = 0; digitNum < numDigits; digitNum++) {
      digits[digitNum] = DASH_IDX;
    }
  } else {
    uint8_t digitNum = 0;
    // Convert all number to positive values
    if (numToShow < 0) {
      digits[0] = DASH_IDX;
      digitNum = 1;  // Skip the first iteration
      numToShow = -numToShow;
    }
    // Find all digits for base's representation, starting with the most
    // significant digit
    for (; digitNum < numDigits; digitNum++) {
      int16_t factor = powersOfBase[numDigits - 1 - digitNum];
      digits[digitNum] = numToShow / factor;
      numToShow -= digits[digitNum] * factor;
    }
    // Find unnnecessary leading zeros and set them to BLANK
    if (decPlaces < 0) decPlaces = 0;

    for (digitNum = 0; digitNum < (numDigits - 1 - decPlaces); digitNum++) {
      if (digits[digitNum] == 0) {
        digits[digitNum] = BLANK_IDX;
        // Exit once the first non-zero number is encountered
      } else if (digits[digitNum] <= 9) {
        break;
      }
    }
  }
}

/**
 * Маппинг массива чисел на массив байт, которыми выводятся эти числа на экран
 */
void setDigitCodes(const uint8_t digits[], int8_t decPlaces) {
  // Set the digitCode for each digit in the display
  for (uint8_t digitNum = 0; digitNum < numDigits; digitNum++) {
    digitCodes[digitNum] = digitCodeMap[digits[digitNum]];
    // Set the decimal point segment
    if (decPlaces >= 0) {
      if (digitNum == numDigits - 1 - decPlaces) {
        digitCodes[digitNum] &= digitCodeMap[PERIOD_IDX];
      }
    }
  }
}

/**
 * Вывести число из глобальной переменной digitCodes на экран
 */
void drawDigits() {
  uint8_t chasPosMask1 = 0b10000000;
  uint8_t chasPosMask2 = 0b01000000;
  uint8_t chasPosMask3 = 0b00100000;
  uint8_t chasPosMask4 = 0b00010000;
  drawSymbol(digitCodes[0], chasPosMask4);
  drawSymbol(digitCodes[1], chasPosMask3);
  drawSymbol(digitCodes[2], chasPosMask2);
  drawSymbol(digitCodes[3], chasPosMask1);
}

void clearDisplay() {
  drawSymbol(0b00000000, 0b00000000);
}

/* 
 * Вывести символ в одной позиции экрана
 */
void drawSymbol(uint8_t input, uint8_t symbolPos) {
  digitalWrite(LATCH_PIN, 0);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, input);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, symbolPos);
  digitalWrite(LATCH_PIN, 1);
}
