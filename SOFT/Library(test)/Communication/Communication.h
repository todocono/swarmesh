/*
Communication.h - Library for Swarmesh - Created on 6/17/20

////////////////////////IMPORTANT////////////////////////////
AsyncUDP is NOT declare in this file
////////////////////////////////////////////////////////////

*/

#ifndef Communication_h
#define Communication_h
#include "Arduino.h"
#include <ArduinoJson.h>
#include "Wifi.h"
#include "Destination.h"

class Communication
{
  private:
    AsyncUDP _udp;
    char *_ssid;
    char *_password;

  public:
    Communication(char *ssid, char *password);
    void wifi_init();
    void udp_init(Destinations dst, const char* ID, int *POS, int *ORI, DynamicJsonDocument &jInfo);
    char *json_creator(int *POS);
};

#endif