int val;
int LED_1 = 4;
int LED_2 = 5;
int LED_3 = 6;
bool led_1_state = false;
bool led_2_state = false;
bool led_3_state = false;

void setup() {
  Serial.begin(9600);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
}

void loop() {

  if (Serial.available()) {
    val = Serial.read();
    if (val == '1') {    
      led_1_state = !led_1_state;
    }
    if (val == '2') {    
      led_2_state = !led_2_state;
    }
    if (val == '3') {    
      led_3_state = !led_3_state;
    }
  }  

  if (led_1_state) {
    digitalWrite(LED_1, HIGH);
  } else {
    digitalWrite(LED_1, LOW);
  }

  if (led_2_state) {
    digitalWrite(LED_2, HIGH);
  } else {
    digitalWrite(LED_2, LOW);
  }

  if (led_3_state) {
    digitalWrite(LED_3, HIGH);
  } else {
    digitalWrite(LED_3, LOW);
  }
}