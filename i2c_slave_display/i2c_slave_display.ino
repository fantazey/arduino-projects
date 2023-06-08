#include <Wire.h>
#define DISPLAY_I2C_ADDRESS 0x29
#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DATA_PIN PB1   // display DIO pin connected
#define CLOCK_PIN PB3  // display SCLK pin connected
#define LATCH_PIN PB4  // display RCLK pin connected
#else
#define DATA_PIN 2   // display DIO pin connected
#define CLOCK_PIN 4  // display SCLK pin connected
#define LATCH_PIN 3  // display RCLK pin connected
#endif


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

// данные для отображения
float numberToShowF;
int16_t numberToShowI = 0;
uint8_t decimalPos;
bool showInteger = true;
bool needDataUpdate = true;

int8_t numDigits = MAXNUMDIGITS;   // число знаков на экране
uint8_t digitCodes[MAXNUMDIGITS];  // хранение битмап для каждой позиции для отображения на экране
static const int16_t powersOf10[] = {
  1,      // 10^0
  10,     // 10^1
  100,    // 10^2
  1000,   // 10^3
  10000,  // 10^4
};


void setup() {
  Wire.begin(DISPLAY_I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  // инициализируем пины экрана
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
}

void receiveEvent(int dataSize) {
  uint8_t index = 0;
  int16_t intResult = 0;
  uint8_t decPlaces = 0;
  uint16_t pow10;
  // 250 - начало передачи числа
  // два байта числа для отображения ((n >> 8) & 0xff) и (n & 0xff)
  // 251 - конец передачи числа
  // один байт - знаков после запятой - 1 байт
  // 252 - конец передачи
  // для передачи числа 12.34 будет такая последовательность 250-4-210-251-2-252
  uint8_t buff[6] = {0,0,0,0,0,0};
  while (Wire.available() > 0) {
    buff[index++] = Wire.read();
  }
  if (buff[0] != 250 && buff[3] != 251 && buff[5] != 252) {
    numberToShowI = -1;
    showInteger = true;
    needDataUpdate = true;
    return;
  }
  intResult = (((uint16_t)buff[1] << 8) | buff[2]);
  decPlaces = buff[4];
  if (decPlaces == 0) {      
    showInteger = true;
    numberToShowI = intResult;
  } else {      
    pow10 = power10(decPlaces);
    numberToShowF = (float)intResult / pow10;
    decimalPos = decPlaces;
    showInteger = false;      
  }
  needDataUpdate = true;
}

uint32_t t2;
const uint16_t _10sec = 10000;

void loop() {
  if (needDataUpdate) {
    if (showInteger) {
      setNumber(numberToShowI);
    } else {
      setNumberF(numberToShowF, decimalPos);
    }
    needDataUpdate = false;
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

uint16_t power10(uint8_t power) {
  uint16_t res = 1;
  int8_t p = power;
  while(p > 0) {
    res = res * 10;
    p--;
  }  
  return res;
}