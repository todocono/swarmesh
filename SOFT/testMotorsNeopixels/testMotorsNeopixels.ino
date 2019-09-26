//#include <Servo.h>

#include <FastLED.h>

FASTLED_USING_NAMESPACE

// FastLED "100-lines-of-code" demo reel, showing just a few
// of the kinds of animation patterns you can quickly and easily
// compose using FastLED.  
//
// This example also shows one easy way to define multiple
// animations patterns and have them automatically rotate.
//
// -Mark Kriegsman, December 2014

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

#define DATA_PIN    25
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120
\

//pins for ultrasound
#define trigPin D0
#define echoPin D1

//pins for motors
#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13

// constants won't change :
const long TIME = 500;           // interval at which to blink (milliseconds)


void setup()
{
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  //FastLED.addLeds<LED_TYPE,DATA_PIN,CLK_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
  Serial.begin(115200);
  pinMode( 12, OUTPUT);
  pinMode( 13, OUTPUT);
  pinMode( 14, OUTPUT);
  pinMode( 27, OUTPUT);


  pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  Serial.println("starting...");

  //setup channel 0 with frequency 312500 Hz
  sigmaDeltaSetup(0, 312500);
  // sigmaDeltaSetup(1, 312500);
  // sigmaDeltaSetup(2, 312500);
  sigmaDeltaSetup(3, 312500);

  sigmaDeltaAttachPin(12, 0);
  // sigmaDeltaAttachPin(13,1);
  // sigmaDeltaAttachPin(14,2);
  sigmaDeltaAttachPin(27, 3);
  //initialize channel 0 to off
  sigmaDeltaWrite(0, 0);
  sigmaDeltaWrite(3, 0);
}


// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

 
void loop()
{
    // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
 
  int speed0 = 254;
  int speed1 = 254;
  Serial.println("MOTOR 0...");
  motor0 (speed0, 0);
  delay (TIME);
  motor0 (speed0, 1);
  delay (TIME);
  motor0 (0, 0);

  Serial.println("MOTOR 1...");
  motor1 (speed1, 0);
  delay (TIME);
  motor1 (speed1, 1);
  delay (TIME);
  motor1 (0, 0);


  Serial.println("BOTH MOTORS...");
  motor0 (speed1, 0);
  motor1 (speed1, 0);
  delay (TIME);
  motor0 (speed1, 1);
  motor1 (speed1, 1);
  delay (TIME);
  motor0 (0, 0);
  motor1 (0, 0);
}

void motor0 ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRA, HIGH);
    sigmaDeltaWrite(0, 255 - speed);
  }
  else {
    digitalWrite(DIRA, LOW);
    sigmaDeltaWrite(0, speed);
  }
}

void motor1 ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRB, HIGH);
    sigmaDeltaWrite(3, 255 - speed);
  }
  else {
    digitalWrite(DIRB, LOW);
    sigmaDeltaWrite(3, speed);
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
