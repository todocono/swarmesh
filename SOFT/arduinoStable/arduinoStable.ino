////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////THE ID OF THE ROBOT NEEDS TO BE SET ACCORDING TO THE ARUCO CODE IT CARRIES/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 7
//SET ACCORDING TO THE ARUCO CODE

int *POS;
int ORI;
int TURN;
int DIST;
int STATE = 0;
const char *ID;

class Destinations
{
  private:
    int *_dst;
    int *_init_pos;
    int **_dst_lst;
    int _lst_size;
    void _del_dst_lst(int idx);

  public:
    Destinations();
    void set_class();
    void select_dst(int *POS);
    void load_dst(DynamicJsonDocument &jTask, int *POS);
    void proceed_dst(DynamicJsonDocument &jDst, int *POS);
    int* get_dst();
    int arrive_dst(int *POS);
};

Destinations::Destinations()
{
  int **_dst_lst;
  int *_dst;
  int *_init_pos;
  int _lst_size;
}

void Destinations::set_class() {
  _dst = (int *)malloc(sizeof(int) * 2);
  _init_pos = (int *)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) _dst[i] = -1;
}

void Destinations::_del_dst_lst(int idx)
{
  //  _dst[0] = _dst_lst[idx][0];
  //  _dst[1] = _dst_lst[idx][1];
  // swap the value at index with the last element
  _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
  _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
  // modify _dst_lst and get rid of the smallest element
  _lst_size--;
  Serial.print("Reduced List Size: ");
  Serial.println(_lst_size);
  int **new_ptr = (int **)malloc(sizeof(int *) * (_lst_size));
  for (int i = 0; i < _lst_size; i++)
  {
    new_ptr[i] = (int *)malloc(sizeof(int) * 2);
    for (int j = 0; j < 2; j++)
    {
      new_ptr[i][j] = _dst_lst[i][j];
    }
    free(_dst_lst[i]);
  }
  free(_dst_lst);
  _dst_lst = new_ptr;
}

void Destinations::load_dst(DynamicJsonDocument &jTask, int *POS)
{
  for (int i = 0; i < 2; i ++) _init_pos[i] = POS[i];
  _lst_size = jTask["Num"];
  Serial.print("Task size: ");
  Serial.println(_lst_size);
  _dst_lst = (int **)malloc(sizeof(int *) * _lst_size);
  // load json destination coordinates into the two-dimensional array
  for (int i = 0; i < _lst_size; i++)
  {
    _dst_lst[i] = (int *)malloc(sizeof(int) * 2);
    for (int j = 0; j < 2; j++)
    {
      _dst_lst[i][j] = jTask["Task"][i][j];
    }
  }
  // automatically acquire the next destination by calling the function
  select_dst(POS);
}

void Destinations::select_dst(int *POS)
{
  // select the destination with the least manhattan distance
  int x = _dst_lst[0][0];
  int y = _dst_lst[0][1];
  int idx = 0;
  int dist = (abs(x - POS[0]) + abs(y - POS[1]));
  for (int i = 0; i < _lst_size; i++)
  {
    x = _dst_lst[i][0];
    y = _dst_lst[i][1];
    int new_dist = (abs(x - POS[0]) + abs(y - POS[1]));
    if (new_dist < dist)
    {
      dist = new_dist;
      idx = i;
    }
  }
  for (int i = 0; i < 2; i ++) _dst[i] = _dst_lst[idx][i];
  _del_dst_lst(idx);
}

void Destinations::proceed_dst(DynamicJsonDocument &jDst, int *POS)
{
  int *dst = (int *)malloc(sizeof(int) * 2);
  dst[0] = jDst["Pos"][0];
  dst[1] = jDst["Pos"][1];
  Serial.println(_lst_size);
  if (_lst_size > 0)
  {
    // proceed to the next least distance point
    if (dst[0] == _dst[0] && dst[1] == _dst[1])
    {
      select_dst(POS);
    }
    else
    {
      // get the id of the element
      int idx = -1;
      for (int i = 0; i < _lst_size; i++)
      {
        if (_dst_lst[i][0] == dst[0] && _dst_lst[i][1] == dst[1]) idx = i;
        Serial.println("matched");
      }
      //      filter out destination of robots going back to their origin
      if (idx >= 0) _del_dst_lst(idx);
    }
  }
  else
  {
    // run out of available Destinations
    // go back to original point
    //    only doing so when the last destination conflicts the destination taken by others
    if (dst[0] == _dst[0] && dst[1] == _dst[1]) {
      for (int i = 0; i < 2; i ++) _dst[i] = _init_pos[i];
    }
  }
}

int* Destinations::get_dst() {
  int* new_ptr = (int*) malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) new_ptr[i] = _dst[i];
  return new_ptr;
}

int Destinations::arrive_dst(int *POS)
{
  if (POS[0] == _dst[0] && POS[1] == _dst[1]) return 1;
  return 0;
}

struct Tick
{
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void IRAM_ATTR isr()
{
  encoder1.numberTicks += 1;
  encoder1.tickOn = true;
}

void IRAM_ATTR isr2()
{
  encoder2.numberTicks += 1;
  encoder2.tickOn = true;
}

const char *ssid = "nowifi";
const char *password = "durf2020";

AsyncUDP udp;
Destinations dstc;

void setup()
{
  //  SET UP THE ID OF THE ROBOT HERE
  ID = "5";
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);
  ledcSetup(1, 12000, 8);
  ledcSetup(2, 12000, 8);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
  POS = (int *)malloc(sizeof(int) * 2);
  dstc.set_class();
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    }
  }
  //  listening to both task and current position on this channel
  if (udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001))
  {
    udp.onPacket([](AsyncUDPPacket packet) {
      DynamicJsonDocument jInfo(1024);
      deserializeJson(jInfo, packet.data());
      const int Purpose = jInfo["Purpose"];
      switch (Purpose)
      {
        case 1:
          {
            Serial.println("Position received");

            //            not move if it can't get its position in json document
            if (!jInfo[ID][0][0] && !jInfo[ID][0][1]) break;
            POS[0] = jInfo[ID][0][0];
            POS[1] = jInfo[ID][0][1];
            ORI = jInfo[ID][1];
            // move only if it has received tasks
            int* ptr = dstc.get_dst();
            int arrive = dstc.arrive_dst(POS);
            Serial.println(ptr[0]);
            Serial.println(ptr[1]);
            Serial.println();
            if (ptr[0] != -1 && !arrive)
            {
              actionDecoder(ORI, POS, ptr);
            }
            else if (arrive)
            {
              char jsonStr[80];
              //              jsonCreator(jsonStr);
              const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(2);
              DynamicJsonDocument doc(capacity);
              doc["Purpose"] = 3;
              JsonArray Pos = doc.createNestedArray("Pos");
              Pos.add(POS[0]);
              Pos.add(POS[1]);sss
              serializeJson(doc, jsonStr);
              doc.clear();
              udp.writeTo((const uint8_t*) jsonStr, strlen(jsonStr), IPAddress(224, 3, 29, 1), 10001);
            }
            free(ptr);
          }
          break;
        case 2:
          Serial.println("Tasks received");
          STATE = 1;
          dstc.load_dst(jInfo, POS);
          break;
        case 3:
          //        other robots have arrived at their destinations
          //        need to check whether the destinations are the same as its own destination
          //        move only when it self is not at the destination
          if (!dstc.arrive_dst(POS)) dstc.proceed_dst(jInfo, POS);
      }
      jInfo.clear();

    });
  }
}

void loop()
{
  if (STATE == 1)
  {
    forward(1000);
  }
  else if (STATE == 2)
  {
    turnLeft(TURN);
  }
  else if (STATE == 3)
  {
    turnRight(TURN);
  }
}

//void dstSelector(int ORI, int *POS, int *DST, int *DST_LST, DynamicJsonDocument &jTask)
//{
//  //  const int* dst = jTask["Task"];
//  int x = jTask["Task"][0][0];
//  int y = jTask["Task"][0][1];
//  int idx = 0;
//  int dist = (abs(x - POS[0]) + abs(y - POS[1]));
//  const int len = jTask["Num"];
//  for (int i = 0; i < len; i++)
//  {
//    //  calc manhattan distance
//    int x = jTask["Task"][i][0];
//    int y = jTask["Task"][i][1];
//    int new_dist = (abs(x - POS[0]) + abs(y - POS[1]));
//    Serial.println(new_dist);
//    if (new_dist < dist)
//    {
//      dist = new_dist;
//      idx = i;
//    }
//  }
//  DST[0] = jTask["Task"][idx][0];
//  DST[1] = jTask["Task"][idx][1];
//}

//void jsonCreator(char* jsonStr) {
//  const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(2);
//  DynamicJsonDocument doc(capacity);
//  doc["Purpose"] = 3;
//  JsonArray Pos = doc.createNestedArray("Pos");
//  Pos.add(POS[0]);
//  Pos.add(POS[1]);
//  serializeJson(doc, jsonStr);
//  doc.clear();
//}

void actionDecoder(int ORI, int *POS, int *DST)
{
  //  calculate the robots' absolute position
  int x = DST[0] - POS[0];
  int y = DST[1] - POS[1];
  TURN = 90;
  if (abs(ORI) <= 5)
  {
    if (y >= 1)
    {
      STATE = 3;
      TURN = 180;
    }
    else if (y <= -1)
    {
      STATE = 1;
      DIST = y * 1000;
    }
    else
    {
      if (x >= 1)
      {
        STATE = 3;
      }
      else if (x <= -1)
      {
        STATE = 2;
      }
    }
  }
  else if (abs(ORI) >= 175)
  {
    if (y >= 1)
    {
      STATE = 1;
      DIST = abs(y) * 1000;
    }
    else if (y <= -1)
    {
      STATE = 3;
      TURN = 180;
    }
    else
    {
      if (x >= 1)
      {
        STATE = 2;
      }
      else if (x <= -1)
      {
        STATE = 3;
      }
    }
  }
  else if (ORI >= 85 && ORI <= 95)
  {
    if (x >= 1)
    {
      STATE = 1;
      DIST = x * 1000;
    }
    else if (x <= 1)
    {
      STATE = 3;
      TURN = 180;
    }
    else
    {
      if (y >= 1)
      {
        STATE = 2;
      }
      else if (y <= -1)
      {
        STATE = 3;
      }
    }
  }
  else if (ORI >= -95 && ORI <= -85)
  {
    if (x >= 1)
    {
      STATE = 3;
      TURN = 180;
    }
    else if (x <= -1)
    {
      STATE = 1;
      DIST = abs(x) * 1000;
    }
    else
    {
      if (y >= 1)
      {
        STATE = 3;
      }
      else if (y <= -1)
      {
        STATE = 2;
      }
    }
  }
  else
  {
    int remain = ORI % 90;
    if (abs(remain) >= 45)
    {
      STATE = (remain < 0) ? 2 : 3;
      TURN = 90 - abs(remain);
    }
    else
    {
      STATE = (remain > 0) ? 2 : 3;
      TURN = abs(remain);
    }
  }
}

void forward(int dist)
{
  if (encoder1.numberTicks < dist)
  {
    if (encoder1.numberTicks > encoder2.numberTicks)
    {
      motorA(200, 0);
      motorB(0, 0);
    }
    else if (encoder1.numberTicks < encoder2.numberTicks)
    {
      motorA(0, 0);
      motorB(200, 0);
    }
    else
    {
      motorA(200, 0);
      motorB(200, 0);
    }
  }
  else
  {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    STATE = 0;
  }
}

void turnLeft(int deg)
{
  if (encoder1.numberTicks <= PULSE * deg)
  {
    //    if ( encoder1.numberTicks > encoder2.numberTicks) {
    //      motorA (100, 0);
    //      motorB (0, 0);
    //    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
    //      motorA (0, 0);
    //      motorB (100, 1);
    //    } else {
    motorA(100, 0);
    motorB(100, 1);
    //    }
  }
  else
  {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    STATE = 0;
  }
}

void turnRight(int deg)
{
  if (encoder1.numberTicks <= PULSE * deg)
  {
    //       if ( encoder1.numberTicks > encoder2.numberTicks) {
    //        motorA (100, 1);
    //        motorB (0, 0);
    //       } else if ( encoder1.numberTicks < encoder2.numberTicks){
    //        motorA (0, 0);
    //        motorB (100, 0);
    //      } else {
    motorA(100, 1);
    motorB(100, 0);
    //      }
  }
  else
  {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    STATE = 0;
  }
}

void motorA(int speed, int direction)
{
  if (direction)
  {
    digitalWrite(DIRA, HIGH);
    ledcWrite(1, 255 - speed);

    // sigmaDeltaWrite(0, 255 - speed);
  }
  else
  {
    digitalWrite(DIRA, LOW);
    //sigmaDeltaWrite(0, speed);
    ledcWrite(1, speed);
  }
}

void motorB(int speed, int direction)
{
  if (direction)
  {
    digitalWrite(DIRB, HIGH);
    //sigmaDeltaWrite(3, 255 - speed);
    ledcWrite(2, 255 - speed);
  }
  else
  {
    digitalWrite(DIRB, LOW);
    //sigmaDeltaWrite(3, speed);
    ledcWrite(2, speed);
  }
}
