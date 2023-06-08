#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <BMP280.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <DFRobot_DHT11.h>


#ifdef __AVR_ATmega328P__
#define PIN_LED_CONTROL 9//PIN_PB1  // output PWM led control
#define PIN_DHT_11 10//PIN_PB2
#define PIN_GREEN_LIGHT 8//PIN_PB0
#define PIN_SERIAL_YELLOW_LIGHT 7//PIN_PD7
#define PIN_SSRX 11 //PIN_PB3
#define PIN_SSTX 12 //PIN_PB4
#else
#define PIN_LED_CONTROL 9  // output PWM led control
#define PIN_DHT_11 8
#define PIN_GREEN_LIGHT 7
#define PIN_SERIAL_YELLOW_LIGHT 6
#define PIN_SSRX 2
#define PIN_SSTX 3
#endif


SoftwareSerial sSerial(PIN_SSRX, PIN_SSTX);
#define serialCommunication sSerial 
#define _DEBUG_K(...) Serial.print(__VA_ARGS__)
#define _DEBUG_V(...) Serial.println(__VA_ARGS__)

#ifdef __DEBUG__
#define serialCommunication Serial 
SoftwareSerial sSerial(PIN_SSRX, PIN_SSTX);
#define _DEBUG_K(...) //sSerial.print(__VA_ARGS__)
#define _DEBUG_V(...) //sSerial.println(__VA_ARGS__)
#endif

#define serialDataStart digitalWrite(PIN_SERIAL_YELLOW_LIGHT, HIGH)
#define serialDataStop digitalWrite(PIN_SERIAL_YELLOW_LIGHT, LOW)
// #define commandDataStart digitalWrite(PIN_GREEN_LIGHT, HIGH)
// #define commandDataStop digitalWrite(PIN_GREEN_LIGHT, LOW)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define to16(h, l) (((uint16_t)h << 8) | l)
#define to8h(u16i) ((u16i >> 8) & 0xff)
#define to8l(u16i) ((uint8_t)u16i & 0xff)
//
#define I2CADDRESS_RTC1307 0x50  // 0x68
#define I2CADDRESS_BMP280 0x76
#define I2CADDRESS_BH1750 0x23
#define I2CADDRESS_DISPLAY 0x29

//
// PC4 - A4 - SDA - i2c data pin
// PC5 - A5 - SCL - i2c clock pin
// eeprom data indexes
#define IDX_PAINTED_H 1
#define IDX_PAINTED_L 2
#define IDX_UNPAINTED_H 3
#define IDX_UNPAINTED_L 4
#define IDX_DISPLAY_MODE 5
#define IDX_LIGHT_MODE 6
#define IDX_LIGHT_LUX_MAX_VALUE 7
#define IDX_LIGHT_FIXED_VALUE 8


// данные для передачи в модуль экрана
uint16_t numberBuffer;
uint8_t decPlaces;

// delays
const uint16_t _10secDelay = 10000;
const uint16_t _5secDelay = 5000;
const uint16_t _1secDelay = 1000;

// timers
uint32_t displayModeTimer;    // таймер текущего режима отображения
uint32_t lightSensorTimer;    // таймер чтения сенсора GY-302
uint32_t bmpTimer;            // таймер чтения сенсора BMP280
uint32_t rtcTimer;       // таймер чтения времени DS1307
uint32_t dhtTimer;       // таймер чтения времени DS1307
uint32_t updateDisplayTimer;  // таймер обновления данных на экране

// режимы "экрана"
#define D_MAX_MODE 11
#define D_SHIFT_START 4 // стартовый индекс режима при обычном шафле всех режимов
#define D_YEAR 1 // показать год
#define D_MIN_SEC 2 // показать минуты и секунды
#define D_DHT_TEMP 3 // показать температуру с DHT11
#define D_DATE_MON 4 // показать день и месяц
#define D_HOUR_MIN 5 // показать часы и минуты
#define D_BMP_TEMP 6  // показать температуру с BMP280
#define D_BMP_PRESS 7 // показать давление с BMP280
#define D_DHT_HUM 8  // показать влажность с DHT11
#define D_LUX 9  // показать интенсивность света
#define D_PAINTED 10 // показать количество покрашеных миниатюр
#define D_UNPAINTED 11 // показать количество непокрашеных миниатюр
uint8_t displayMode = D_SHIFT_START;
bool fixedDisplay = false;  // отключить смену режимов экрана

#define L_OFF 0 // свет выкл
#define L_DYNAMIC 1 // динамический свет зависящий от датчика интенсивности
#define L_FADE 2 // мигающий свет
#define L_FIXED 3 // фиксированная яркость установленная руками
#define L_LOW 4 // тусклый свет
#define L_MID 5 // средний свет
#define L_HIGH 6 // полная яркость
uint8_t lightMode;
uint8_t fixedBrightness;

// measures
DFRobot_DHT11 DHT;
uint16_t dhtHumidity;
int16_t dhtTemperature;

uint16_t lightToDisplay;
float light;  // интенсивность света

BMP280 bmp280(I2CADDRESS_BMP280);
int16_t temperature;
uint16_t pressure;

uint16_t paintedMiniatures = 35;
uint16_t unpaintedMiniatures = 432;

tmElements_t tm;

// internals
bool needUpdateDigits = true;
float lightLuxToBrightnessMult = 3.18;
uint8_t brightness = 40;
uint8_t fadeAmount = 2;

// serial input
uint32_t parsedNumber;
const uint8_t serialBuffSize = 64;
uint8_t serialBuff[serialBuffSize];
uint8_t serialBuffReadSize;
bool startCommandReceived = false;
bool allDataReceived = false;


void setup() {
  delay(1000);
  Serial.begin(9600);
  serialCommunication.begin(9600);
  // инициализируем отладочные пины
  pinMode(PIN_GREEN_LIGHT, OUTPUT);
  pinMode(PIN_SERIAL_YELLOW_LIGHT, OUTPUT);
  digitalWrite(PIN_GREEN_LIGHT, LOW);
  digitalWrite(PIN_SERIAL_YELLOW_LIGHT, LOW);
  
  // инициализируем пин управления светом
  pinMode(PIN_LED_CONTROL, OUTPUT);
  
  
  // инициализируем dht11
  pinMode(PIN_DHT_11, INPUT);

  // инициализируем шину I2C
  Wire.begin();
  // запускаем датчик интенсивности освещения с адресом по-умолчанию
  _BH1750begin();
  // запускаем датчик BMP280 с адресом 0x76  
  bmp280.begin();

  paintedMiniatures = to16(EEPROM.read(IDX_PAINTED_H), EEPROM.read(IDX_PAINTED_L));
  unpaintedMiniatures = to16(EEPROM.read(IDX_UNPAINTED_H), EEPROM.read(IDX_UNPAINTED_L));
  displayMode = EEPROM.read(IDX_DISPLAY_MODE);
  if (displayMode == 0) {
    displayMode = D_SHIFT_START;
    fixedDisplay = false;
  } else {
    fixedDisplay = true;
  }      
  lightMode = EEPROM.read(IDX_LIGHT_MODE);
  lightLuxToBrightnessMult = 255.0 / (EEPROM.read(IDX_LIGHT_LUX_MAX_VALUE));
  fixedBrightness = EEPROM.read(IDX_LIGHT_FIXED_VALUE);

  // debug
  displayMode = D_SHIFT_START;

  digitalWrite(PIN_GREEN_LIGHT, HIGH);
  _DEBUG_V(F("START"));
}

void loop() {  
  readSerialData();
  
  if (millis() - lightSensorTimer > _10secDelay) {
    _BH1750light();
    lightSensorTimer = millis();
  }

  
  if (millis() - bmpTimer > _10secDelay) {
    temperature = bmp280.getTemperature() * 100;
    pressure = bmp280.getPressure() * 0.075006156;
    bmpTimer = millis();
  }
  
  if (millis() - dhtTimer > _10secDelay) {
    DHT.read(PIN_DHT_11);    
    dhtHumidity = DHT.humidity < 255 ? DHT.humidity : -1;
    dhtTemperature = DHT.temperature < 255 ? DHT.temperature : -1;
    dhtTimer = millis();
  }

  uint16_t timeDelay = displayMode == D_MIN_SEC ? _1secDelay : _5secDelay;
  if (millis() - rtcTimer > timeDelay) {
    RTC.read(tm);
    rtcTimer = millis();
  }

  if (millis() - updateDisplayTimer > timeDelay) {
    needUpdateDigits = true;
    updateDisplayTimer = millis();
  }

  // переключаем режимы экрана
  if (millis() - displayModeTimer > _10secDelay) {
    displayModeTimer = millis();
    if (!fixedDisplay) {
      displayMode++;
      needUpdateDigits = true;
    }
    if (displayMode > D_MAX_MODE) {
      displayMode = D_SHIFT_START;
    }
  }


  // рисуем на экране
  if (needUpdateDigits) {
    showData();
  }

  // выствляем яркость ангара
  lightUpHangar();
}

void showData() {
  switch (displayMode) {
    case D_YEAR:
      setNumber(tmYearToCalendar(tm.Year), 0);
      break;
    case D_DATE_MON:
      setNumber(tm.Day * 100 + tm.Month, 0);
      break;
    case D_MIN_SEC:
      setNumber(tm.Minute * 100 + tm.Second, 2);
      break;    
    case D_HOUR_MIN:
      setNumber(tm.Hour * 100 + tm.Minute, 2);
      break;
    case D_BMP_TEMP:
      setNumber(temperature, 2);
      break;
    case D_BMP_PRESS:
      setNumber(pressure, 1);
      break;
    case D_DHT_HUM:
      setNumber(dhtHumidity, 0);
      break;
    case D_DHT_TEMP:
      setNumber(dhtTemperature, 0);
      break;
    case D_LUX:
      setNumber(lightToDisplay, 1);
      break;
    case D_PAINTED:
      setNumber(paintedMiniatures, 0);
      break;
    case D_UNPAINTED:
      setNumber(unpaintedMiniatures, 0);
      break;
  }
  needUpdateDigits = false;
  drawDigits();
}

void lightUpHangar() {
  switch (lightMode) {
    case L_OFF:
      brightness = 0;
      break;
    case L_DYNAMIC:
      brightness = 255 - min(255, (light * lightLuxToBrightnessMult));
      break;
    case L_FADE:
      brightness = brightness + fadeAmount;
      if (brightness <= 30 || brightness >= 200) {
        fadeAmount = -fadeAmount;
      }
      break;
    case L_FIXED:
      brightness = fixedBrightness;
      break;
    case L_LOW:
      brightness = 10;
      break;
    case L_MID:
      brightness = 100;
      break;
    case L_HIGH:
      brightness = 255;
      break;
  }
  analogWrite(PIN_LED_CONTROL, brightness);
}


void setNumber(int16_t numToShow, uint8_t dec) {
  numberBuffer = numToShow;
  decPlaces = dec;
}


void drawDigits() {
  Wire.beginTransmission(I2CADDRESS_DISPLAY);
  Wire.write(250);
  Wire.write((numberBuffer >> 8) & 0xff);
  Wire.write((uint8_t)numberBuffer & 0xff);
  Wire.write(251);
  Wire.write(decPlaces);
  Wire.write(252);
  Wire.endTransmission();
}

bool _BH1750begin() {
  Wire.beginTransmission(I2CADDRESS_BH1750);
  Wire.write(0x10);  // Continuously H-Resolution Mode
  if (Wire.endTransmission() == 0) {
    delay(180);  // Wait to complete 1st H-resolution mode measurement. (max. 180ms.)
    return true;
  }
  return false;
}

void _BH1750light() {
  int n = Wire.requestFrom(I2CADDRESS_BH1750, 2);
  if (n != 2) {
    return;
  }
  uint16_t val = 0;
  val = Wire.read();
  val <<= 8;
  val |= Wire.read();
  light = val / 1.2;
  lightToDisplay = light * 10;
}


void readSerialData() {
  uint8_t c;  
  uint8_t index = 0;
  uint8_t index2 = 0;
  if (serialCommunication.available() == 0) {
    return;
  }
  _DEBUG_K(F("available serial data size: "));
  _DEBUG_V(serialCommunication.available());
  serialDataStart;
  delay(1000);
  while (serialCommunication.available() > 0) {  
    if (serialBuffReadSize >= serialBuffSize) {
      serialBuffReadSize=0;
    }
    c = serialCommunication.read();
    if (c == '#') {
      allDataReceived = true;
      startCommandReceived = false;
      continue;
    }
    if (c == '@') {      
      serialBuffReadSize = 0;
      startCommandReceived = true;
      continue;
    }
    if (startCommandReceived)
      serialBuff[serialBuffReadSize++] = c;
  }
  serialDataStop;
  if (serialBuffReadSize == 0 || !allDataReceived) {
    return;
  }

  // парсим данные пришедшие из последовательного порта
  bool updateTime = false;
  tmElements_t newTime;  
  delay(1000);
  while (index < serialBuffReadSize) {
    _DEBUG_K(F("index: "));
    _DEBUG_V(serialBuff[index++]);
  }
  index=0;
  RTC.read(newTime);
  while (index < serialBuffReadSize) {
    c = serialBuff[index];    
    index++;
    String commandChars("HMSymdDLlBPU");
    if (commandChars.indexOf(c) >= 0) {
      _DEBUG_K(F("command accepted: "));
      _DEBUG_V(c);
      index2 = readIntFromBuff(serialBuff, index, serialBuffReadSize);
      if (index == index2) {
        continue;
      }      
      index = index2;      
      _DEBUG_K(F("data parsed: "));
      _DEBUG_V(parsedNumber);
    }
    uint8_t parsedByte = (uint8_t)parsedNumber;
    if (c == 'H') {
      newTime.Hour = parsedByte;
      updateTime = true;
    }
    if (c == 'M') {
      newTime.Minute = parsedByte;
      updateTime = true;
    }
    if (c == 'S') {
      newTime.Second = parsedByte;
      updateTime = true;
    }
    if (c == 'y') {
      newTime.Year = CalendarYrToTm(2000 + parsedByte);
      updateTime = true;
    }
    if (c == 'm') {
      newTime.Month = parsedByte;
      updateTime = true;
    }
    if (c == 'd') {
      newTime.Day = parsedByte;
      updateTime = true;
    }
    //==========================================================
    if (c == 'D') {
      displayMode = parsedByte;
      EEPROM.write(IDX_DISPLAY_MODE, displayMode);
      if (displayMode == 0) {
        displayMode = D_SHIFT_START;
        fixedDisplay = false;
      } else {
        fixedDisplay = true;
      }      
    }
    if (c == 'L') {
      lightMode = parsedByte;
      EEPROM.write(IDX_LIGHT_MODE, lightMode);
    }
    if (c == 'l') {
      lightLuxToBrightnessMult = 255.0 / parsedByte;
      EEPROM.write(IDX_LIGHT_LUX_MAX_VALUE, parsedByte);
    }
    if (c == 'B') {
      fixedBrightness = parsedByte;
      EEPROM.write(IDX_LIGHT_FIXED_VALUE, fixedBrightness);      
    }
    if (c == 'P') {
      paintedMiniatures = parsedNumber;
      EEPROM.write(IDX_PAINTED_H, to8h(paintedMiniatures));
      EEPROM.write(IDX_PAINTED_L, to8l(paintedMiniatures));
    }
    if (c == 'U') {
      unpaintedMiniatures = parsedNumber;
      EEPROM.write(IDX_UNPAINTED_H, to8h(unpaintedMiniatures));
      EEPROM.write(IDX_UNPAINTED_L, to8l(unpaintedMiniatures));
    }    
  }
  if (updateTime) {
    RTC.write(newTime);
  }
  allDataReceived = false;
  serialBuffReadSize = 0;
  index = 0;
  while(index<serialBuffSize) {
    serialBuff[index++] = 0;
  }
}

uint8_t readIntFromBuff(uint8_t* buff, uint8_t startIndex, uint8_t size) {
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