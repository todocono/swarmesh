#include <FastLED.h>
#include "WiFi.h"
#include "AsyncUDP.h"

#define DATA_PIN 25
#define NUM_LEDS 2

CRGB leds[NUM_LEDS];

#define DARK_PURPLE CRGB(51, 6, 98)
#define MID_PURPLE CRGB(87, 6, 140)
#define LIGHT_PURPLE CRGB(137, 0, 225)
#define BRIGHTNESS 250
int brightness_ctr = 0;
int goUp = 1;
int STATE = 0;
int ctr = 0;

char *ssid = "nowifi";
char *password = "durf2020";

AsyncUDP udp;

void setup()
{
  delay(3000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    }
  }
  if (udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001))
  {
    udp.onPacket([](AsyncUDPPacket packet) {
      DynamicJsonDocument jInfo(1024);
      deserializeJson(jInfo, packet.data());
      const int Purpose = jInfo["Purpose"];
      switch (Purpose)
      {
      case 1:
        STATE = 1;
      case 0:
        STATE = 0;
      }
    });
  }
}

void loop()
{
  if (STATE == 2)
  {
    if (goUp)
      brightness_ctr++;
    else
      brightness_ctr--;
    if (brightness_ctr == 250)
    {
      goUp = 0;
    }
    else if (brightness_ctr == 0)
    {
      goUp = 1;
      STATE = 3;
    }
    FastLED.setBrightness(brightness_ctr);
    //    leds[0] = DARK_PURPLE;
    //    leds[1] = DARK_PURPLE;
    //    leds[0] = MID_PURPLE;
    //    leds[1] = MID_PURPLE;
    leds[0] = LIGHT_PURPLE;
    leds[1] = LIGHT_PURPLE;
    //    FastLED.show();
    //    delay(500);
    //    leds[0]=CRGB::Black;
    //    leds[1]=CRGB::Black;
    FastLED.show();
    delay(2);
    //    FastLED.delay(1);
  }
  else if (STATE == 3)
  {
    delay(200);
    STATE = 1;
  }
  else if (STATE == 1)
  {
    delay(200);
    STATE = 2;
  }
  else
  {
    FastLED.setBrightness(0);
    FastLED.show();
  }
}
