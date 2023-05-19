
#include <Arduino.h>
#include <Wire.h>
#include <ArtronShop_BH1750.h>
#include <Adafruit_BMP280.h>

#define LED_CONTROL 9

ArtronShop_BH1750 bh1750(0x23, &Wire); // Non Jump ADDR: 0x23, Jump ADDR: 0x5C

Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();


uint32_t measureTimer = 0;
uint32_t measureTimerDelay = 3000;
float light;

void setup() {
  
  Serial.begin(9600);
  pinMode(LED_CONTROL, OUTPUT);  
  Wire.begin();
  bh1750.begin();

  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("setup ok");
}

void loop() {
  if (millis() - measureTimer > measureTimerDelay) {
    doMeasure();
    measureTimer = millis();
  }
  if (light > 254) {
    reset();
  } else {
    analogWrite(LED_CONTROL, 255 - (uint8_t)light);  
  }
}

void doMeasure() {
  light = bh1750.light();
  Serial.print("Light: ");
  Serial.print(light);
  Serial.println(" lx");

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
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
