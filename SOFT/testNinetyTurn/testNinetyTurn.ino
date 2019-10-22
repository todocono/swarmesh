//pins for motors
#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 312

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

int count = 0;

void setup()
{

  pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);

  Serial.begin(115200);

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
  delay(2000);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
}



void loop()
{

  int speed0 = 250;
  int speed1 = 250;
  int dist = 5000;
  //Serial.println("BOTH MOTORS...");
  // make a 180 degrees turn
  forward(2500);
  turnRight(45);
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


void turnRight(int deg){
  int num = deg / 45;
  while (encoder1.numberTicks <= PULSE * num) {
       if ( encoder1.numberTicks > encoder2.numberTicks) {
        motorA (100, 1);
        motorB (0, 0);
       } else if ( encoder1.numberTicks < encoder2.numberTicks){
        motorA (0, 0);
        motorB (100, 0);
      } else {
        motorA(100, 1);
        motorB(100, 0);
      }
       Serial.printf("Motor 1: %d", encoder1.numberTicks);
       Serial.printf("Motor 2: %d", encoder2.numberTicks);
       Serial.println();
  } 
  motorA(0, 0);
  motorB(0, 0);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
}

void forward(int dist){
  while (encoder1.numberTicks <= dist) {
       if ( encoder1.numberTicks > encoder2.numberTicks) {
        motorA (200, 0);
        motorB (0, 0);
       } else if ( encoder1.numberTicks < encoder2.numberTicks){
        motorA (0, 0);
        motorB (200, 0);
      } else {
        motorA(200, 0);
        motorB(200, 0);
      }
       Serial.printf("Motor 1: %d", encoder1.numberTicks);
       Serial.printf("Motor 2: %d", encoder2.numberTicks);
       Serial.println("forward");
  } 
  motorA(0, 0);
  motorB(0, 0);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
}

void turnLeft(int deg){
  int num = deg / 45;
  while (encoder1.numberTicks <= PULSE * num) {
       if ( encoder1.numberTicks > encoder2.numberTicks) {
        motorA (100, 0);
        motorB (0, 0);
       } else if ( encoder1.numberTicks < encoder2.numberTicks){
        motorA (0, 0);
        motorB (100, 1);
      } else {
        motorA(100, 0);
        motorB(100, 1);
      }
       Serial.printf("Motor 1: %d", encoder1.numberTicks);
       Serial.printf("Motor 2: %d", encoder2.numberTicks);
  } 
  motorA(0, 0);
  motorB(0, 0);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
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
