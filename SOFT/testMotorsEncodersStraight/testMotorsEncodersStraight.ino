
//pins for motors llyggghkgl
#define PWMA 27hjfghgfhjgf
#define DIRA 14
#define PWMB 12
#define DIRB 13

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


void setup()
{

  Serial.begin(115200);

  // pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  // pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);

  ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
  ledcAttachPin(PWMB, 2);
  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);


  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);


  Serial.println("starting...");
}



void loop()
{

  int speed0 = 125;
  int speed1 = 125;


  //Serial.println("BOTH MOTORS...");
  if ( encoder1.numberTicks > encoder2.numberTicks) {
    motorA (speed1, 0);
    motorB (0, 0);
  } else {
    motorA (0, 0);
    motorB (speed1, 0);
  }
  //  delay (TIME);
  //  motor0 (speed1, 1);
  //  motor1 (speed1, 1);
  //  delay (TIME);
  //  motor0 (0, 0);
  //  motor1 (0, 0);

  //for debugging
  //  if (encoder1.tickOn ) {
  //    Serial.printf("Encoder 1 has been %u times", encoder1.numberTicks);
  //    Serial.printf(" and Encoder 2 %u \n", encoder2.numberTicks);
  //    encoder1.tickOn = false;
  //  }
  //  if (encoder2.tickOn ) {
  //    Serial.printf("Encoder 1 has been %u times", encoder1.numberTicks);
  //    Serial.printf(" and Encoder 2 %u \n", encoder2.numberTicks);
  //    encoder2.tickOn = false;
  //  }


}


void motorA ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRA, HIGH);
    ledcWrite(1, 255 - speed);

    // sigmaDeltaWrite(0, 255 - speed);
  }
  else {
    digitalWrite(DIRA, LOW);
    //sigmaDeltaWrite(0, speed);
    ledcWrite(1, speed);
  }
}

void motorB ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRB, HIGH);
    //sigmaDeltaWrite(3, 255 - speed);
    ledcWrite(2, 255 - speed);
  }
  else {
    digitalWrite(DIRB, LOW);
    //sigmaDeltaWrite(3, speed);
    ledcWrite(2, speed);
  }
}
