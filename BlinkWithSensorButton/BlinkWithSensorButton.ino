/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

int lastState = LOW;      // the previous state from the input pin
int currentState;         // the current reading from the input pin
int active = 0;
// the setup function runs once when you press reset or power the board
void setup() {  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(5, INPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(7, LOW);   // turn the LED off by making the voltage LOW
  digitalWrite(8, LOW);   // turn the LED off by making the voltage LOW
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
  currentState = digitalRead(5);
  if (currentState == HIGH && lastState == LOW) {
    active++;
    lastState = currentState;
  }
  if (currentState == LOW && lastState == HIGH) {    
    lastState = currentState;
  }
  if (active > 3) {
    active = 0;
  }
  if (active == 0) {
    digitalWrite(6, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(7, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(8, LOW);  // turn the LED on (HIGH is the voltage level)    
  }
  if (active == 1) {
    digitalWrite(7, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(8, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(6, HIGH);  // turn the LED on (HIGH is the voltage level)    
  }
  if (active == 2) {
    digitalWrite(6, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(8, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(7, HIGH);  // turn the LED on (HIGH is the voltage level)    
  }
  if (active == 3) {
    digitalWrite(6, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(7, LOW);   // turn the LED off by making the voltage LOW
    digitalWrite(8, HIGH);  // turn the LED on (HIGH is the voltage level)    
  }
}
