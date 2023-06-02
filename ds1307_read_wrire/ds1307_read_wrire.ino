#include <DFRobot_DS1307.h>

/* Constructor */
DFRobot_DS1307 DS1307;

uint16_t timeBuff[7] = {5, 1, 7, 6, 9, 9, 2021};

void setup()
{
  Serial.begin(9600);
  DS1307.begin();
  DS1307.setTypeTime(DS1307.eYR, 2000);
  DS1307.stop();
  DS1307.setTime(timeBuff);
  DS1307.start();
}

uint32_t readTimeTimer;
uint16_t readTimeDelay = 6000;

void loop()
{
  if(millis() - readTimeTimer > readTimeDelay) {
    DS1307.getTime(timeBuff);
    /**
   *  Get the time from rtc module and convert it to uint16_t
   *  getTimeBuff Array for buffering the obtained time, uint16_t *
   *    getTimeBuff[0] for eSEC type, range: 00-59
   *    getTimeBuff[1] for eMIN type, range: 00-59
   *    getTimeBuff[2] for eHR type, range: 00-23
   *    getTimeBuff[3] for eDOW type, range: 01-07
   *    getTimeBuff[4] for eDATE type, range: 01-31
   *    getTimeBuff[5] for eMTH type, range: 01-12
   *    getTimeBuff[6] for eYR type, range: 2000-2099
   */
    Serial.println(timeBuff[2] * 100 + timeBuff[1]);
    readTimeTimer = millis();
  }
  

}