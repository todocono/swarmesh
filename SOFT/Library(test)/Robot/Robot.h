/*
Robot.h - Library for Swarmesh - Created on 6/17/20

The constructor of Robot would automatically construct other classes 
including Destinations, Locomotion, Communication

Need to declare another variable AsyncUDP
In setup(), please remember to type in the following
////////////////////////////////////////////////////
if (_udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001))
  {
    _udp.onPacket([](AsyncUDPPacket packet) {
      DynamicJsonDocument jInfo(1024);
      deserializeJson(jInfo, packet.data());
      robot.udp_init(jInfo);
      if (robot.robot_action()) {
        char *jsonStr = robot.json_creator();
        _udp.writeTo((const uint8_t*) jsonStr, strlen(jsonStr), IPAddress(224, 3, 29, 1), 10001);
        free(jsonStr);
      }
    });
  }
////////////////////////////////////////////////////
Because the UDP can only be initialized outside
*/

#ifndef Robot_h
#define Robot_h
#include "Arduino.h"
#include <ArduinoJson.h>
#include "Locomotion.h"
#include "Destinations.h"
#include "Communication.h"

class Robot
{
private:
    int *_POS;
    int _ORI;
    int _TURN;
    int _STATE;
    int _battery;
    const char *_ID;
    Locomotion _loc;
    Destinations _dst;
    Communication _com;

public:
    Robot(Encoder *encoder1, Encoder *encoder2, char *ssid, char *password, const char *ID);
    void robot_init();
    void execute_main();
    void update_battery(int level);
    void decode_action(int *pos, int *dst);
    void udp_init(DynamicJsonDocument &jInfo);
    const char *get_id();
    char *json_creator();
    int *get_pos();
    int robot_action();
    int get_battery();
    int get_state();
    int get_ori();
};

#endif