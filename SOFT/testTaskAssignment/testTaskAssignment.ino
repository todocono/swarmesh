#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 7

int ORI;
int POS[2];
int ID_int;
char *ID;
int counter = 0;
int action;

class Destinations
{
private:
  int _lst_size;
  int _bid;
  int _dst[2];
  int _init_pos[2];
  int **_dst_lst;
  void _del_dst_lst(int idx);

public:
  Destinations();
  void select_dst(int *POS);
  void load_dst(DynamicJsonDocument &jTask, int *POS);
  int proceed_bid(DynamicJsonDocument &jDst, int *POS);
  int arrive_dst(int *POS);
  int bid_state();
  int *get_dst();
  int *make_bid();
};

Destinations::Destinations()
{
  _bid = 0;
  for (int i = 0; i < 2; i++)
    _dst[i] = -1;
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
  for (int i = 0; i < 2; i++)
    _init_pos[i] = POS[i];
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
  // swap this destination to the front of the array
  int temp[2];
  temp[0] = _dst_lst[0][0];
  temp[1] = _dst_lst[0][1];
  for (int i = 0; i < 2; i++)
  {
    _dst_lst[0][i] = _dst_lst[idx][i];
    _dst_lst[idx][i] = temp[i];
  }
}

int Destinations::proceed_bid(DynamicJsonDocument &jDst, int *POS)
{
  int dst[2];
  dst[0] = jDst["Pos"][0];
  dst[1] = jDst["Pos"][1];
  if (_lst_size - 1 > 0)
  {
    if (dst[0] == _dst_lst[0][0] && dst[1] == _dst_lst[0][1])
    {
      _del_dst_lst(0);
      select_dst(POS);
    }
    else
    {
      // get the id of the element
      int idx = -1;
      for (int i = 0; i < _lst_size; i++)
      {
        if (_dst_lst[i][0] == dst[0] && _dst_lst[i][1] == dst[1])
          idx = i;
      }
      //      filter out destination of robots going back to their origin
      if (idx >= 0)
        _del_dst_lst(idx);
    }
    return 0;
  }
  // the last robot broadcasting its selection is also the signal for all robots to act
  // those didn't make bid would not act
  return bid_state();
}

int *Destinations::get_dst()
{
  int *new_ptr = (int *)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i++)
    new_ptr[i] = _dst[i];
  return new_ptr;
}

int *Destinations::make_bid()
{
  if (!bid_state())
  {
    _bid = 1;
    for (int i = 0; i < 2; i++)
      _dst[i] = _dst_lst[0][i];
    _del_dst_lst(0);
    return get_dst();
  }
  return nullptr;
}

int Destinations::arrive_dst(int *POS)
{
  if (POS[0] == _dst[0] && POS[1] == _dst[1])
    return 1;
  return 0;
}

int Destinations::bid_state()
{
  return _bid;
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
Destinations dst;

void setup()
{
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
//  ID_int = ID.toInt();
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
        if (!jInfo[ID][0][0] && !jInfo[ID][0][1])
          break;
        POS[0] = jInfo[ID][0][0];
        POS[1] = jInfo[ID][0][1];
        ORI = jInfo[ID][1];
      case 2:
        dst.load_dst(jInfo, POS);
        break;
      case 3:
        action = dst.proceed_bid(jInfo, POS);
        counter++;
        break;
      }
      jInfo.clear();
    });
  }
}

void loop()
{
  if (!action)
  {
    if (counter == ID_int - 1)
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
      udp.writeTo((const uint8_t *)jsonStr, strlen(jsonStr), IPAddress(224, 3, 29, 1), 10001);
    }
  }
  else
  {
    Serial.println("act now!");
  }
}
