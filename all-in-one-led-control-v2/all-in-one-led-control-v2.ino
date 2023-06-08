#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <BMP280.h>

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)

#define LED_CONTROL PB1  // output PWM led control

#else

#define LED_CONTROL 9  // output PWM led control
#endif

//
#define IDX_CHANGED 1
#define IDX_YEAR 2
#define IDX_MONTH 3
#define IDX_DAY 4
#define IDX_HOUR 5
#define IDX_MINUTE 6
#define IDX_SECOND 7
#define IDX_DHT_HUMIDITY 8
#define IDX_DHT_TEMPERATURE 9
//
#define IDX_PAINTED_H 2
#define IDX_PAINTED_L 3
#define IDX_UNPAINTED_H 4
#define IDX_UNPAINTED_L 5
#define IDX_LIGHT_MODE 6
#define IDX_LIGHT_LUX_MAX_VALUE 7
#define IDX_DISPLAY_MODE 8
//
#define min(a, b) ((a) < (b) ? (a) : (b))
#define to16(h, l) (((uint16_t)h << 8) | l)
//
// #define I2CADDRESS_RTC1307 0x50  // и еще какойто
#define I2CADDRESS_BMP280 0x76
#define I2CADDRESS_BH1750 0x23
#define I2CADDRESS_DISPLAY 0x29
#define I2CADDRESS_DHT11 0x30

//
// A4 - SDA - i2c data pin
// A5 - SCL - i2c clock pin
// данные для передачи в модуль экрана
uint16_t numberBuffer;
uint8_t decPlaces;
// delays
const uint16_t _10secDelay = 10000;
const uint16_t _5secDelay = 5000;
// timers
uint32_t displayModeTimer;    // таймер текущего режима отображения
uint32_t configTimer;         // таймер чтения конфига + dht11
uint32_t lightSensorTimer;    // таймер чтения сенсора GY-302
uint32_t bmpTimer;            // таймер чтения сенсора BMP280
uint32_t readTimeTimer;       // таймер чтения времени из DS1307
uint32_t updateDisplayTimer;  // таймер чтения времени из DS1307

#define D_MAX_MODE 11
#define D_SHIFT_START 4
#define D_YEAR 1
#define D_MIN_SEC 2
#define D_DHT_TEMP 3
#define D_DATE_MON 4
#define D_HOUR_MIN 5
#define D_BMP_TEMP 6
#define D_BMP_PRESS 7
#define D_DHT_HUM 8
#define D_LUX 9
#define D_PAINTED 10
#define D_UNPAINTED 11
uint8_t displayMode = D_MIN_SEC;
bool fixedDisplay = false;

// const uint16_t powers10[2] = {10, 100};

#define L_OFF 0
#define L_DYNAMIC 1
#define L_FADE 2
#define L_FIXED 3
#define L_LOW 4
#define L_MID 5
#define L_HIGH 6
uint8_t lightMode;
// measures
uint16_t dhtHumidity;
int16_t dhtTemperature;
uint16_t lightToDisplay;
float light;  // интенсивность света

// BMP280 bmp280(I2CADDRESS_BMP280);
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


void setup() {
  Serial.begin(9600);
  // инициализируем пин управления светом
  pinMode(LED_CONTROL, OUTPUT);
  // инициализируем шину I2C
  Wire.begin();
  // запускаем датчик интенсивности освещения с адресом по-умолчанию
  _BH1750begin();
  // запускаем датчик BMP280 с адресом 0x76
  // bmp280.begin();
  Serial.println("Start");
}

void loop() {
  if (millis() - configTimer > 2 * _10secDelay) {
    // readConfigBlock1();
    readConfigBlock2();
    configTimer = millis();
  }

  if (millis() - lightSensorTimer > _10secDelay) {
    _BH1750light();
    lightSensorTimer = millis();
  }

  // if (millis() - bmpTimer > _10secDelay) {
  //   readBMP280Sensor();
  //   bmpTimer = millis();
  // }

  if (millis() - readTimeTimer > _5secDelay) {
    RTC.read(tm);
    readTimeTimer = millis();
  }

  // переключаем режимы экрана
  // if (millis() - displayModeTimer > _10secDelay) {
  //   displayModeTimer = millis();
  //   if (!fixedDisplay) {
  //     displayMode++;
  //     needUpdateDigits = true;
  //   }
  //   if (displayMode > D_MAX_MODE) {
  //     displayMode = D_SHIFT_START;
  //   }
  // }
  if (millis() - updateDisplayTimer > _5secDelay) {
    needUpdateDigits = true;
    updateDisplayTimer = millis();
  }
  // выствляем яркость ангара
  lightUpHangar();
  // рисуем на экране
  if (needUpdateDigits) {
    showData();
  }
}

void readConfigBlock2() {
  // read second block of data
  Wire.beginTransmission(I2CADDRESS_DHT11);
  Wire.write(250);
  Wire.write(2);
  Wire.endTransmission();
  delay(1000);
  uint8_t configBuff[12] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  uint8_t index = 0;
  uint8_t n = Wire.requestFrom(I2CADDRESS_DHT11, 12);
  delay(1000);
  Serial.print(F("Request config 2: "));
  uint8_t bbb = 0;
  while (Wire.available() > 0) {
    bbb = Wire.read();
    configBuff[index++] = bbb;
    Serial.println(bbb);
  }
  Serial.println("end config block 2");
  if (configBuff[0] != 252 || configBuff[11] != 253 || configBuff[IDX_CHANGED] == 0) {
    return;
  }
  paintedMiniatures = to16(configBuff[IDX_PAINTED_H], configBuff[IDX_PAINTED_L]);
  unpaintedMiniatures = to16(configBuff[IDX_UNPAINTED_H], configBuff[IDX_UNPAINTED_L]);
  lightMode = configBuff[IDX_LIGHT_MODE];
  lightLuxToBrightnessMult = 255.0 / configBuff[IDX_LIGHT_LUX_MAX_VALUE];
  fixedDisplay = configBuff[IDX_DISPLAY_MODE] != 0;
  displayMode = fixedDisplay ? configBuff[IDX_DISPLAY_MODE] : displayMode;
}

void readConfigBlock1() {
  // set first config block
  Wire.beginTransmission(I2CADDRESS_DHT11);
  Wire.write(250);
  Wire.write(1);
  // read first block of data
  Wire.endTransmission();

  uint8_t configBuff[12] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  uint8_t index = 0;
  Wire.requestFrom(I2CADDRESS_DHT11, 12);
  // delay(1000);
  Serial.print(F("Request config 1: "));
  while (Wire.available() > 0) {
    configBuff[index++] = Wire.read();
    Serial.println(configBuff[index-1]);
  }
  Serial.println("end config block 1");
  if (configBuff[0] != 250 || configBuff[11] != 251) {
    return;
  }
  if (configBuff[IDX_CHANGED] == 0) {
    dhtHumidity = configBuff[IDX_DHT_HUMIDITY];
    dhtTemperature = configBuff[IDX_DHT_TEMPERATURE];
    temperature = configBuff[IDX_DHT_TEMPERATURE];
    return;
  }
  //parse first block of data
  RTC.read(tm);
  tm.Hour = configBuff[IDX_HOUR];
  tm.Minute = configBuff[IDX_MINUTE];
  tm.Second = configBuff[IDX_SECOND];
  tm.Year = CalendarYrToTm(2000 + configBuff[IDX_YEAR]);
  tm.Month = configBuff[IDX_MONTH];
  tm.Day = configBuff[IDX_DAY];
  tmElements_t tm2;
  RTC.read(tm2);
  if (makeTime(tm2) < makeTime(tm)) {
    RTC.write(tm);
  }
}

void readBMP280Sensor() {
  // temperature = bmp280.getTemperature() * 100;
  // pressure = bmp280.getPressure() * 0.075006156;
  temperature = 2;
  pressure = 3;
}

void showData() {
  switch (displayMode) {
    case D_YEAR:
      setNumber(tmYearToCalendar(tm.Year), 0);
      break;
    case D_MIN_SEC:
      setNumber(tm.Minute * 100 + tm.Second, 0);
      break;
    case D_DATE_MON:
      setNumber(tm.Day * 100 + tm.Month, 0);
      break;
    case D_HOUR_MIN:
      setNumber(tm.Hour * 100 + tm.Minute, 0);
      break;
    case D_BMP_TEMP:
      // setNumberF(temperature, 2);
      setNumber(temperature, 2);
      break;
    case D_BMP_PRESS:
      setNumber(pressure, 1);
      break;
    case D_DHT_HUM:
      setNumber(dhtHumidity, 2);
      break;
    case D_DHT_TEMP:
      setNumber(dhtTemperature, 2);
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
    case L_FIXED:
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
  analogWrite(LED_CONTROL, brightness);
}


void setNumber(int16_t numToShow, uint8_t dec) {
  numberBuffer = numToShow;
  decPlaces = dec;
}

// void setNumberF(float numToShow, uint8_t dec) {
//   numberBuffer = numToShow;// * powers10[dec];
//   decPlaces = dec;
// }


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

void _BH1750begin() {
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
    return -1.0;
  }
  uint16_t val = 0;
  val = Wire.read();
  val <<= 8;
  val |= Wire.read();
  light = val / 1.2;
  lightToDisplay = light * 10;
}