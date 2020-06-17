#include <Arduino.h>
#include <ArduinoJson.h>
#include "Wifi.h"
#include "AsyncUDP.h"
#include "Communication.h"

Communication::Communication(char *ssid, char *password)
{
  _udp;
  _ssid = ssid;
  _password = password;
}

void Communication::wifi_init()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(_ssid, _password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    }
  }
}

void Communication::udp_init(Destinations dst, const char* ID, int *POS, int *ORI, DynamicJsonDocument &jInfo)
{
  const int Purpose = jInfo["Purpose"];
  switch (Purpose)
  {
    case 1:
      {
        //   only for updating current position
        Serial.println("Position received");
        // not move if it can't get its position in json document
        if (!jInfo[ID][0][0] && !jInfo[ID][0][1]) break;
        POS[0] = jInfo[ID][0][0];
        POS[1] = jInfo[ID][0][1];
        *ORI = jInfo[ID][1];
        Serial.println(POS[0]);
        Serial.println(POS[1]);
        Serial.println();
        // move only if it has received tasks
        //        int *ptr = dst.get_dst();
        //         int arrive = dst.arrive_dst(POS);
        // Serial.println(ptr[0]);
        // Serial.println(ptr[1]);
        // Serial.println();
        // if (ptr[0] != -1 && !arrive)
        // {
        //   // actionDecoder(ORI, POS, ptr);
        //   Serial.println("Not arrived");
        // }
        // else if (arrive)
        // {
        //   //          self has arrived
        //   //          write udp to the rest of the group
        //   return 1;
        // }
        // free(ptr);
      }
      break;
    case 2:
      Serial.println("Tasks received");
      dst.load_dst(jInfo, POS);
      break;
    case 3:
      //        other robots have arrived at their destinations
      //        need to check whether the destinations are the same as its own destination
      //        move only when it self is not at the destination
      if (!dst.arrive_dst(POS))
        dst.proceed_dst(jInfo, POS);
  }
  jInfo.clear();
}

char *Communication::json_creator(int *POS)
{
  char jsonStr[80];
  //              jsonCreator(jsonStr);
  const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);
  doc["Purpose"] = 3;
  JsonArray Pos = doc.createNestedArray("Pos");
  Pos.add(POS[0]);
  Pos.add(POS[1]);
  serializeJson(doc, jsonStr);
  doc.clear();
  return jsonStr;
}