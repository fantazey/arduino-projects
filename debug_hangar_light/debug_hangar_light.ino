int DEBUG_PIN_TINY_OUT = 2; // читаем выход с мк
int DEBUG_PIN_TINY_IN = 3; // читаем вход на мк
int DEBUG_PIN_KEY = 4; // сигнал на ключ


int LED_TINY_OUT = 7; // пришел сигнал с выхода мк
int LED_TINY_IN = 8; // пришел сигнал с кнопки (вход мк)
int LED_KEY = 9;  //  пришел сигнал на ключ


bool tiny_out_state = false;
bool tiny_in_state = false;
bool key_state = false;

int state1 = LOW;
int state2 = LOW;
int state3 = LOW;

void setup() {  
  pinMode(DEBUG_PIN_TINY_OUT, INPUT);
  pinMode(DEBUG_PIN_TINY_IN, INPUT);
  pinMode(DEBUG_PIN_KEY, INPUT);
  pinMode(LED_TINY_OUT, OUTPUT);
  pinMode(LED_TINY_IN, OUTPUT);
  pinMode(LED_KEY, OUTPUT);
  digitalWrite(LED_TINY_OUT, LOW);    
  digitalWrite(LED_TINY_IN, LOW);    
  digitalWrite(LED_KEY, LOW);      
}

void loop() {
  state1 = digitalRead(DEBUG_PIN_TINY_OUT);
  if (state1 == HIGH) {
    tiny_out_state = true;
  }
  
  state2 = digitalRead(DEBUG_PIN_TINY_IN);
  if (state2 == HIGH) {
    tiny_in_state = true;
  }
  
  state3 = digitalRead(DEBUG_PIN_KEY);
  if (state3 == HIGH) {
    key_state = true;
  }

  if (tiny_out_state == true) {
    digitalWrite(LED_TINY_OUT, HIGH);    
  }
  if (tiny_in_state == true) {
    digitalWrite(LED_TINY_IN, HIGH);    
  }
  if (key_state == true) {
    digitalWrite(LED_KEY, HIGH);    
  }
}
