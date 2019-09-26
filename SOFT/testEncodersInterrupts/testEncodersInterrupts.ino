struct Tick {
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void IRAM_ATTR isr() {
  encoder1.numberTicks += 1;
  encoder1.tickOn = true;
}
void IRAM_ATTR isr2() {
  encoder2.numberTicks += 1;
  encoder2.tickOn = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);
}

void loop() {
  if (encoder1.tickOn ) {
      Serial.printf("Encoder 1 has been %u times", encoder1.numberTicks);
      Serial.printf(" and Encoder 2 %u \n", encoder2.numberTicks);
      encoder1.tickOn = false;
  }
  if (encoder2.tickOn ) {
      Serial.printf("Encoder 1 has been %u times", encoder1.numberTicks);
      Serial.printf(" and Encoder 2 %u \n", encoder2.numberTicks);
      encoder2.tickOn = false;
  }
  //Detach Interrupt after 1 Minute
//  static uint32_t lastMillis = 0;
//  if (millis() - lastMillis > 60000) {
//    lastMillis = millis();
//    detachInterrupt(encoder1.PIN);
//  Serial.println("Interrupt Detached!");
//  }
}
