int lastState = LOW;      // the previous state from the input pin
int currentState;         // the current reading from the input pin
int mode = 0; // 0 - off, 1 - low glowing, 2 - mid glow, 3 bright glow, 4 - fade effect
int maxMode = 4;
int BUTTON_PIN = PB1;
int LED_CONTROL = PB0;
int brightness = 40;    // how bright the LED is
int fadeAmount = 2;    // how many points to fade the LED by

// the setup function runs once when you press reset or power the board
void setup() {  
  delay(500);
  digitalWrite(LED_CONTROL, LOW);   // turn the LED off by making the voltage LOW  
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_CONTROL, OUTPUT);  
  Serial.begin(9600);
}


// the loop function runs over and over again forever
void loop() {
  currentState = digitalRead(BUTTON_PIN);
  if (currentState == HIGH && lastState == LOW) {
    mode++;
    lastState = currentState;
  }
  if (currentState == LOW && lastState == HIGH) {    
    lastState = currentState;
  }  
  if (mode > maxMode) {
    mode = 0;
  }
  switch (mode) {
    case 0:
      reset();
      break;
    case 1:
      lowGlow();
      break;
    case 2:
      midGlow();
      break;
    case 3:
      brightGlow();
      break;
    case 4:
      fadeEffect();
      break;
    default:
      reset();
  }
  delay(50);
}

void fadeEffect() {
    analogWrite(LED_CONTROL, brightness);  
    brightness = brightness + fadeAmount;  
    if (brightness <= 5 || brightness >= 100) {
      fadeAmount = -fadeAmount;
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
    brightness = 40;
    fadeAmount = 2;
}