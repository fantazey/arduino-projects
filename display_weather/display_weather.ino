#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <DS1307RTC.h>


#define DHTPIN 2       // dht data pin
#define DHTTYPE DHT11  // DHT 11
#define LATCH_PIN 4    // display RCLK pin connected
#define CLOCK_PIN 3    // display SCLK pin connected
#define DATA_PIN 5     // display DIO pin connected

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

int8_t numDigits = MAXNUMDIGITS;  // число знаков на экране

uint8_t digitCodes[MAXNUMDIGITS];  // хранение битмап для каждой позиции для отображения на экране
static const int16_t powersOf10[] = {
  1,      // 10^0
  10,     // 10^1
  100,    // 10^2
  1000,   // 10^3
  10000,  // 10^4
};

/**
   * 0 - показываем температуру снаружи
   * - - показываем температуру внутри
   * 1 - показываем влажность
   * 4 - показываем время 
   * 2 - показываем сколько покрашено
   * 3 - показываем сколько не покрашено
   */
uint8_t activeMode = 4;
uint32_t modeTimer;         // таймер текущего режима отображения
uint32_t modeDelay = 5000;  // 10sec

uint32_t dhtReadDelay = 10000;
uint32_t dhtTimer = 0;
float temperature = 0.0;
float humidity = 0.0;
DHT_Unified dht(DHTPIN, DHTTYPE);

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;

tmElements_t tm;
// uint8_t hours = 14;
// uint8_t minutes = 32;

void setup() {
  Serial.begin(9600);
  // инициализируем пины экрана
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  // инициализируем работу с датчиком
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtReadDelay = sensor.min_delay / 100;
  tm.Hour=12;
  tm.Minute=23;
  tm.Second=45;
  tm.Day=15;
  tm.Month=5;
  tm.Year=CalendarYrToTm(2023);
  RTC.write(tm);
  RTC.read(tm);
  Serial.println((uint8_t)tm.Minute);
}

void loop() {
  // читаем серийный порт для обновления данных
  readDataUpdates();
  // измеряем температуру и влажность не слишком часто
  if (millis() - dhtTimer > dhtReadDelay) {
    readDHTSensor();
    dhtTimer = millis();
  }

  // переключаем режимы
  if (millis() - modeTimer > modeDelay) {
    modeTimer = millis();
    // activeMode++;
    if (activeMode > 4) {
      activeMode = 0;
    }
  }

  // рисуем на экране
  showData(activeMode);
}

void readDataUpdates() {
  uint16_t s1 = 99;
  tmElements_t outerTime;
  RTC.read(outerTime);  
  while (Serial.available() > 0) {
    uint8_t c = Serial.read();
    if (c == 'H') {
      s1 = Serial.parseInt();
      outerTime.Hour = s1;
      // if (s1 < 25) {
      //   hours = s1;
      // }
      outerTime.Hour = s1;
      RTC.write(outerTime);
      Serial.print("write hours");
      Serial.println(outerTime.Hour);
    }
    if (c == 'M') {
      s1 = Serial.parseInt();
      outerTime.Minute = s1;
      RTC.write(outerTime);
      Serial.print("write minute");
      Serial.println(outerTime.Minute);
      // if (s1 < 60) {
        // minutes = s1;
      // }
    }
    if (c == 'S') {
      s1 = Serial.parseInt();
      outerTime.Second = s1;
      RTC.write(outerTime);
      // if (s1 < 60) {
        // minutes = s1;
      // }
    }
    if (c == 'y') {
      s1 = Serial.parseInt();
      outerTime.Year = CalendarYrToTm(s1);
      RTC.write(outerTime);
    }
    if (c == 'm') {
      s1 = Serial.parseInt();
      outerTime.Month = s1;
      RTC.write(outerTime);
    }
    if (c == 'd') {
      s1 = Serial.parseInt();
      outerTime.Day = s1;
      RTC.write(outerTime);
    }
    if (c == 'P') {
      s1 = Serial.parseInt();
      paintedMiniatures = s1;
    }
    if (c == 'U') {
      s1 = Serial.parseInt();
      unpaintedMiniatures = s1;
    }
    if (c == '\n') {
      Serial.println("end read data");
      ;
    }
  }
}

void readDHTSensor() {
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    temperature = 0.0;
  } else {
    temperature = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    humidity = 0.0;
  } else {
    humidity = event.relative_humidity;
  }
}


void showData(uint8_t mode) {
  tmElements_t readTime;
  switch (mode) {
    case 0:
      setNumberF(temperature, 2);
      break;
    case 1:
      setNumberF(humidity, 2);
      break;
    case 2:
      setNumber(paintedMiniatures);
      break;
    case 3:
      setNumber(unpaintedMiniatures);
      break;
    case 4:
      RTC.read(readTime);
      // float time = readTime.Hour + ((float)readTime.Minute / 100.0);
      float time = readTime.Minute + ((float)readTime.Second / 100.0);
      setNumberF(time, 2);
  }
  drawDigits();
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
      int32_t factor = powersOfBase[numDigits - 1 - digitNum];
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