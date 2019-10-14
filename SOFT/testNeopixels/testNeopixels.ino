#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 64

#define DATA_PIN 25

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

int n = 0;

void loop() {
  // Turn the LED on, then pause
  leds[n] = CRGB::Blue;
  FastLED.show();
  delay(10);
  // Now turn the LED off, then pause
  leds[n] = CRGB::Black;
  FastLED.show();
  n = n + 1;
  if (n > 64) {
    n = 0;
  }

}
