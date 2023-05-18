#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2 // dht data pin
#define DHTTYPE DHT11  // DHT 11
#define LATCH_PIN 4  // display RCLK pin connected
#define CLOCK_PIN 3  // display SCLK pin connected
#define DATA_PIN 5   // display DIO pin connected

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

#define CHAR_1 0b10000000 // показать число в позиции 1
#define CHAR_2 0b01000000 // показать число в позиции 1
#define CHAR_3 0b00100000 // показать число в позиции 1
#define CHAR_4 0b00010000 // показать число в позиции 1
#define BLANK_IDX 10  // "пробел" индекс должен совпадать с битмапой в 'digitCodeMap'
#define DASH_IDX 11 // "дефис" индекс должен совпадать с битмапой в 'digitCodeMap'
#define PERIOD_IDX 12 // "точка" индекс должен совпадать с битмапой в 'digitCodeMap'
#define UNDERSCORE_IDX 13 // "подчеркивание" индекс должен совпадать с битмапой в 'digitCodeMap'
#define MAXNUMDIGITS 8 // максимальное число знаков

int8_t numDigits = 4; // число знаков на экране

uint8_t digitCodes[MAXNUMDIGITS]; // хранение битмап для каждой позиции для отображения на экране
static const int32_t powersOf10[] = {
  1,  // 10^0
  10,
  100,
  1000,
  10000,
  100000,
  1000000,
  10000000,
  100000000,
  1000000000
};  // 10^9

/**
 * 0 - показываем температуру снаружи
 * 1 - показываем температуру внутри
 * 2 - показываем влажность
 * 3 - показываем время 
 * 4 - показываем сколько покрашено
 * 5 - показываем сколько не покрашено
 */
int8_t activeMode = 0;
uint32_t modeTimer; // таймер текущего режима отображения
uint32_t modeDelay = 10000; // 10sec


uint32_t dhtReadDelay = 10000;
uint32_t dhtTimer = 0;
float temperature = 0.0;
float humidity = 0.0;
DHT_Unified dht(DHTPIN, DHTTYPE);

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;

uint8_t hours = 14;
uint8_t minutes = 32;

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
}


void loop() {
  /**
   * 0 - показываем температуру снаружи
   * 1 - показываем температуру внутри
   * 2 - показываем влажность
   * 3 - показываем время 
   * 4 - показываем сколько покрашено
   * 5 - показываем сколько не покрашено
   */
  uint8_t activeMode = 0;
  if (millis() - dhtTimer > dhtReadDelay) {
    readDHTSensor();
    dhtTimer = millis();
  }
  if (millis() - modeTimer > modeDelay) {
    modeTimer = millis();
    activeMode++;
    if (activeMode > 4) {
      activeMode = 0;      
    }
  }
  showData(activeMode);
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
  switch (mode) {
    case 0:
      setNumberF(temperature, 2);
      break;
    case 1:
      setNumberF(humidity, 2);
      break;
    case 2:
      setNumber(36);
      break;
    case 3:
      setNumber(281);
      break;
    case 4:
      setNumberF(11.00, 2);
  }
  drawDigits(digitCodes);
}

  // void processBluetooth() {
  //   Serial.println("check if bt serial available");
  //   Serial.println(btSerial.available());
  //   if (btSerial.available()<=0) {
  //     return;
  //   }
  //   Serial.println("bt state is:");
  //   Serial.println(digitalRead(BT_STATE));
  // }

  /**
 * Вывести целое число
 */
  void
  setNumber(int32_t numToShow) {  //int32_t
  setNewNum(numToShow, 0);
}

/**
 * Вывести число с нужным количеством десятичных знаков
 */
void setNumberF(float numToShow, int8_t decPlaces) {  //float
  int8_t decPlacesPos = constrain(decPlaces, 0, MAXNUMDIGITS);
  numToShow = numToShow * powersOf10[decPlacesPos];
  // Modify the number so that it is rounded to an integer correctly
  numToShow += (numToShow >= 0.f) ? 0.5f : -0.5f;
  setNewNum((int32_t)numToShow, (int8_t)decPlaces);
}

/**
 * Вывести число с нужным количеством десятичных знаков на экране
 */
void setNewNum(int32_t numToShow, int8_t decPlaces) {
  uint8_t digits[MAXNUMDIGITS];
  findDigits(numToShow, decPlaces, digits);
  setDigitCodes(digits, decPlaces);
  drawDigits(digitCodes);
}

/**
 * Конвертация числа в массив знаков который
 */
void findDigits(int32_t numToShow, int8_t decPlaces, uint8_t digits[]) {
  const int32_t* powersOfBase = powersOf10;
  const int32_t maxNum = powersOfBase[numDigits] - 1;
  const int32_t minNum = -(powersOfBase[numDigits - 1] - 1);

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
 * Вывести числа на экран
 */
void drawDigits(const uint8_t digits[]) {
  drawSymbol4(digits[0]);
  drawSymbol3(digits[1]);
  drawSymbol2(digits[2]);
  drawSymbol1(digits[3]);
}
void drawSymbol4(uint8_t input) {
  drawSymbol(input, CHAR_4);
}
void drawSymbol3(uint8_t input) {
  drawSymbol(input, CHAR_3);
}
void drawSymbol2(uint8_t input) {
  drawSymbol(input, CHAR_2);
}
void drawSymbol1(uint8_t input) {
  drawSymbol(input, CHAR_1);
}
void clearDisplay() {
  drawSymbol(0b00000000, 0b00000000);
}
void drawSymbol(uint8_t input, uint8_t symbolPos) {
  digitalWrite(LATCH_PIN, 0);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, input);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, symbolPos);
  digitalWrite(LATCH_PIN, 1);
}