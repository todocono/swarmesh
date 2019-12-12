#include <FastLED.h>

FASTLED_USING_NAMESPACE

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    25
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          80
#define FRAMES_PER_SECOND  120

//pins for ultrasound
#define trigPin D0
#define echoPin D1

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
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  pinMode( MUXI, INPUT);
  pinMode( MUXA, OUTPUT);
  pinMode( MUXB, OUTPUT);
  pinMode( MUXC, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMA, OUTPUT);
  pinMode(23, OUTPUT);
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

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() {
  // put your main code here, to run repeatedly:
  // use state machine
  // state 1
  // IR measure the strongest radiation
  // record distance and orientation
  // state 2
  // walk towards the objective
  if (state == 0){
    int total_val = 0;
    motorA(0, 0);
    motorB(0, 0);
    for (int i = 0; i < 8; i ++) {
      int pin1 = i & 0x001;
      int pin2 = (i >> 1) & 0x001;
      int pin3 = (i >> 2) & 0x001;
      digitalWrite( MUXA, pin1);
      digitalWrite( MUXB, pin2);
      digitalWrite( MUXC, pin3);
      delay(10);
      int total = 0;
      for (int j = 0; j < 64; j ++){
        int res = analogRead(MUXI);
        total += res;
      }
      
      total /= 64;
      total_val += total;
      int num = pinCheck(i);
      res[num] = total;
    }
    for (int i = 0; i < 8; i ++){
      Serial.print(res[i]);
      Serial.print(" ");
    }
    Serial.println();
    // determine if the radiation is valid or not
    if (total_val > 500 && total_val < 3500){
      // determine the orientation of the radiation
      int max_read = 0;
      int max_idx = 0;
      for (int i = 0; i < 8; i ++){
        if (res[i] > max_read){
          max_read = res[i];
          max_idx = i;
        }
      }
      // calculate the angle
      
      angle = ((max_idx) * 45 + 180) % 360;
      Serial.println(angle);
      Serial.println("------------------------------------------------");
      state += 1;
      delay(10);
    } else if (total_val >= 2500){
      state = 3;
    }
  } else if (state == 1){
    // move towards the target
    if (angle >= 90 && angle <= 270){
      turnLeft(angle - 90);
    } else if (angle < 90){
      turnRight(90 - angle);
    } else {
      turnRight(450 - angle);
    }
  } else if (state == 2){
    // move forward
    forward(1000);
  } else if (state == 3){
    gPatterns[gCurrentPatternNumber]();
    
    // send the 'leds' array out to the actual LED strip
    FastLED.show();
    digitalWrite(23, HIGH);
    motorA(100, 0);
    motorB(100, 1);
  }
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
    if ( encoder1.numberTicks > encoder2.numberTicks) {
      motorA (200, 0);
      motorB (0, 0);
    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
      motorA (0, 0);
      motorB (200, 0);
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
    state = 0;
  }
}

void turnLeft(int deg) {
  int num = deg / 45;
  if (encoder1.numberTicks <= PULSE * num) {
    if ( encoder1.numberTicks > encoder2.numberTicks) {
      motorA (100, 0);
      motorB (0, 0);
    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
      motorA (0, 0);
      motorB (100, 1);
    } else {
      motorA(100, 0);
      motorB(100, 1);
    }
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    state = 2;
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


#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}
