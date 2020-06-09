#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define MUXI 34
#define MUXA 5
#define MUXB 18
#define MUXC 19
#define PULSE 312

int state = 0;
int res[8];
int angle;

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
  // put your setup code here, to run once:
  // set up the adc function
  pinMode( MUXI, INPUT);
  pinMode( MUXA, OUTPUT);
  pinMode( MUXB, OUTPUT);
  pinMode( MUXC, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMA, OUTPUT);
  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);
  ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
  ledcAttachPin(PWMB, 2);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);
  adcAttachPin(MUXI);
  analogSetWidth(12);
  analogSetPinAttenuation(34, ADC_11db);
  Serial.begin(115200);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
}

void loop() {
////////////////////////////10CM = 1000 TURNS////////////////////////////
  if (state == 0){
    forward(1000);
  } else if (state == 1){
    turnLeft(90);
  }
  
  // put your main code here, to run repeatedly:
  // use state machine
  // state 1
  // IR measure the strongest radiation
  // record distance and orientation
  // state 2
  // walk towards the objective
//  if (state == 0){
//    int total_val = 0;
//    motorA(0, 0);
//    motorB(0, 0);
//    for (int i = 0; i < 8; i ++) {
//      int pin1 = i & 0x001;
//      int pin2 = (i >> 1) & 0x001;
//      int pin3 = (i >> 2) & 0x001;
//      digitalWrite( MUXA, pin1);
//      digitalWrite( MUXB, pin2);
//      digitalWrite( MUXC, pin3);
//      delay(10);
//      int total = 0;
//      for (int j = 0; j < 64; j ++){
//        int res = analogRead(MUXI);
//        total += res;
//      }
//      
//      total /= 64;
//      total_val += total;
//      int num = pinCheck(i);
//      res[num] = total;
//    }
//    for (int i = 0; i < 8; i ++){
//      Serial.print(res[i]);
//      Serial.print(" ");
//    }
//    Serial.println();
//    // determine if the radiation is valid or not
//    if (total_val > 500){
//      // determine the orientation of the radiation
//      int max_read = 0;
//      int max_idx = 0;
//      for (int i = 0; i < 8; i ++){
//        if (res[i] > max_read){
//          max_read = res[i];
//          max_idx = i;
//        }
//      }
//      // calculate the angle
//      
////      angle = (max_idx) * 45;
////      Serial.println(angle);
////      Serial.println("------------------------------------------------");
//////      state += 1;
////      delay(10);
//    }
//  } else if (state == 1){
//    // move towards the target
//    if (angle >= 90 && angle <= 270){
//      turnLeft(angle - 90);
//    } else if (angle < 90){
//      turnRight(90 - angle);
//    } else {
//      turnRight(450 - angle);
//    }
//  } else if (state == 2){
//    // move forward
//    forward(1000);
//  }
}

int pinCheck(int num){
  int new_num = num;
  if (num == 7){
    new_num = 5;
  }
  else if (num == 6){
    new_num = 7;
  }
  else if (num == 5){
    new_num = 6;
  }
  return new_num;
}

void turnRight(int deg) {
  int num = deg / 45;
  if (encoder1.numberTicks <= PULSE * num) {
       if ( encoder1.numberTicks > encoder2.numberTicks) {
        motorA (255, 1);
        motorB (0, 0);
       } else if ( encoder1.numberTicks < encoder2.numberTicks){
        motorA (0, 0);
        motorB (255, 0);
      } else {
        motorA(255, 1);
        motorB(255, 0);
      }
       Serial.printf("Motor 1: %d", encoder1.numberTicks);
       Serial.printf("Motor 2: %d", encoder2.numberTicks);
       Serial.println();
    } else {
      motorA(0, 0);
      motorB(0, 0);
      encoder1.numberTicks = 0;
      encoder2.numberTicks = 0;
      state = 2;
    }
}

void forward(int dist) {
  if (encoder1.numberTicks < dist){
    if (dist - encoder1.numberTicks <= 1000){
      int SPD = 0.5 * (dist - encoder1.numberTicks);
      SPD = map(SPD, 0, 1000 * 0.5, 200, 255);
      motorA(SPD, 0);
      motorB(SPD, 0);
    } else {
      motorA(200, 0);
      motorB(200, 0);
    }
//    Serial.printf("Motor 1: %d", encoder1.numberTicks);
//    Serial.printf("Motor 2: %d", encoder2.numberTicks);
//    Serial.println("forward");
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    state = 1;
  }
}

void turnLeft(int deg) {
  int num = deg / 45;
  if (encoder1.numberTicks <= PULSE * num) {
    if ( encoder1.numberTicks > encoder2.numberTicks) {
      motorA (255, 0);
      motorB (0, 0);
    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
      motorA (0, 0);
      motorB (255, 1);
    } else {
      motorA(255, 0);
      motorB(255, 1);
    }
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    state = 0;
  }
  Serial.printf("Motor 1: %d", encoder1.numberTicks);
  Serial.printf("Motor 2: %d", encoder2.numberTicks);
  Serial.println();
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
