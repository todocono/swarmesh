#include "WiFi.h"
#include "AsyncUDP.h"
#include <ArduinoJson.h>

int STATE = 0;

struct Encoder
{
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Encoder encoder1 = {16, 0, false};
Encoder encoder2 = {17, 0, false};

void IRAM_ATTR isr1()
{
  encoder1.numberTicks++;
  encoder1.tickOn = true;
}

void IRAM_ATTR isr2()
{
  encoder2.numberTicks++;
  encoder2.tickOn = true;
}

class Destinations
{
  private:
    int **_dst_lst;
    int *_dst;
    int *_init_pos;
    int _lst_size;
    void _del_dst_lst(int idx);

  public:
    Destinations();
    void dst_init();
    void load_dst(DynamicJsonDocument &jTask, int *POS);
    void select_dst(int *POS);
    void proceed_dst(DynamicJsonDocument &jDst, int *POS);
    int *get_dst();
    int arrive_dst(int *POS);
};

Destinations::Destinations()
{
}

void Destinations::dst_init() {
  _dst = (int *)malloc(sizeof(int) * 2);
  _init_pos = (int *)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) _dst[i] = -1;
}

void Destinations::_del_dst_lst(int idx)
{
  // swap the value at index with the last element
  _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
  _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
  // modify _dst_lst and get rid of the smallest element
  _lst_size--;
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
  Serial.println(POS[0]);
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

class Motor
{
  private:
    Encoder *_encoder;
    int _PWM;
    int _DIR;
    int _IDX;
    uint8_t _PIN;
    void _reset_encoder();

  public:
    Motor(Encoder *encoder, int PWM, int DIR, int IDX);
    void motor_move(int spd, int dir);
    void motor_stop();
    const int get_idx();
    const int get_pwm();
    const int get_dir();
    const int get_tick();
    const uint8_t get_pin();
};

Motor::Motor(Encoder *encoder, int PWM, int DIR, int IDX)
{
  _PIN = encoder->PIN;
  _PWM = PWM;
  _DIR = DIR;
  _IDX = IDX;
  _encoder = encoder;
}

void Motor::motor_move(int spd, int dir)
{
  if (dir)
  {
    digitalWrite(_DIR, HIGH);
    ledcWrite(get_idx(), 255 - spd);
  }
  else
  {
    digitalWrite(_DIR, LOW);
    ledcWrite(get_idx(), spd);
  }
}

void Motor::_reset_encoder()
{
  (*_encoder).numberTicks = 0;
}

void Motor::motor_stop()
{
  _reset_encoder();
  motor_move(0, 0);
}

const int Motor::get_idx()
{
  return _IDX;
}

const int Motor::get_pwm()
{
  return _PWM;
}

const int Motor::get_dir()
{
  return _DIR;
}

const int Motor::get_tick()
{
  return (*_encoder).numberTicks;
}

const uint8_t Motor::get_pin()
{
  return _PIN;
}

class Locomotion
{
  private:
    Motor _motor1;
    Motor _motor2;
    Motor *_motors;
    int _PULSE;
  public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);
    int turn(int deg, char dir);
    int forward(int dist);
    void motor_init();
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor1(encoder1, 27, 14, 1),
  _motor2(encoder2, 12, 13, 2)
{
  _PULSE = 7;
}

int Locomotion::forward(int dist)
{
  Serial.println(_motor1.get_tick());
  if (_motor1.get_tick() < dist)
  {
    _motor1.motor_move(200, 0);
    _motor2.motor_move(200, 0);
    //    if (_motor1.get_tick() > _motor2.get_tick())
    //    {
    //      _motor1.motor_move(200, 0);
    //      _motor2.motor_move(0, 0);
    //    }
    //    else if (_motor1.get_tick() < _motor2.get_tick())
    //    {
    //      _motor1.motor_move(0, 0);
    //      _motor2.motor_move(200, 0);
    //    }
    //    else
    //    {
    //      _motor1.motor_move(200, 0);
    //      _motor2.motor_move(200, 0);
    //    }
    return 0;
  }
  else
  {
    for (int i = 0; i < 2; i++)
      _motors[i].motor_stop();
    return 1;
  }
}

int Locomotion::turn(int deg, char dir)
{
  int num1;
  int num2;
  switch (dir)
  {
    case 'L':
      num1 = 0;
      num2 = 1;
      break;

    case 'R':
      num1 = 1;
      num2 = 0;
      break;
  }
  if (_motor1.get_tick() <= _PULSE * deg)
  {
    //    if ( encoder1.numberTicks > encoder2.numberTicks) {
    //      motorA (100, 0);
    //      motorB (0, 0);
    //    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
    //      motorA (0, 0);
    //      motorB (100, 1);
    //    } else {
    _motor1.motor_move(100, num1);
    _motor2.motor_move(100, num2);
    //    }
    return 0;
  }
  else
  {
    for (int i = 0; i < 2; i++)
      _motors[i].motor_stop();
    return 1;
    // STATE = 0;
  }
}

void Locomotion::motor_init()
{
  _motors = (Motor *)malloc(sizeof(Motor) * 2);
  _motors[0] = _motor1;
  _motors[1] = _motor2;
  for (int i = 0; i < 2; i++)
  {
    pinMode(_motors[i].get_pwm(), OUTPUT);
    pinMode(_motors[i].get_dir(), OUTPUT);
    pinMode(_motors[i].get_pin(), INPUT_PULLUP);
    //    attachInterrupt(_motors[i].get_pin(), _motors[i].isr(), FALLING);
    if (i)
      attachInterrupt(_motors[i].get_pin(), isr1, FALLING);
    else
      attachInterrupt(_motors[i].get_pin(), isr2, FALLING);
    ledcAttachPin(_motors[i].get_pwm(), _motors[i].get_idx());
    ledcSetup(_motors[i].get_idx(), 12000, 8);
  }
}

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
        int x = jInfo[ID][0][0];
        int y = jInfo[ID][0][1];
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

Robot::Robot(Encoder *encoder1, Encoder *encoder2, char *ssid, char *password, const char *ID) : _loc(encoder1, encoder2),
  _com(ssid, password)
{
  _ID = ID;
  _STATE = 0;
}

void Robot::robot_init()
{
  _dst.dst_init();
  _com.wifi_init();
  _loc.motor_init();
  Serial.begin(115200);
  _POS = (int*)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i ++) _POS[i] = -1;
  Serial.println(_POS[0]);
}

void Robot::execute_main()
{
  int turn;
  switch (_STATE)
  {
    case 1:
      turn = _loc.forward(1000);
      break;
    case 2:
      turn = _loc.turn(_TURN, 'L');
      break;
    case 3:
      turn = _loc.turn(_TURN, 'R');
      break;
  }
  if (turn) _STATE = 0;
}

void Robot::update_battery(int level)
{
  _battery = level;
}

void Robot::decode_action(int *POS, int *DST)
{
  int x = DST[0] - POS[0];
  int y = DST[1] - POS[1];
  _TURN = 90;
  if (abs(_ORI) <= 5)
  {
    if (y >= 1)
    {
      _STATE = 3;
      _TURN = 180;
    }
    else if (y <= -1)
    {
      _STATE = 1;
    }
    else
    {
      if (x >= 1)
      {
        _STATE = 3;
      }
      else if (x <= -1)
      {
        _STATE = 2;
      }
    }
  }
  else if (abs(_ORI) >= 175)
  {
    if (y >= 1)
    {
      _STATE = 1;
    }
    else if (y <= -1)
    {
      _STATE = 3;
      _TURN = 180;
    }
    else
    {
      if (x >= 1)
      {
        _STATE = 2;
      }
      else if (x <= -1)
      {
        _STATE = 3;
      }
    }
  }
  else if (_ORI >= 85 && _ORI <= 95)
  {
    if (x >= 1)
    {
      _STATE = 1;
    }
    else if (x <= 1)
    {
      _STATE = 3;
      _TURN = 180;
    }
    else
    {
      if (y >= 1)
      {
        _STATE = 2;
      }
      else if (y <= -1)
      {
        _STATE = 3;
      }
    }
  }
  else if (_ORI >= -95 && _ORI <= -85)
  {
    if (x >= 1)
    {
      _STATE = 3;
      _TURN = 180;
    }
    else if (x <= -1)
    {
      _STATE = 1;
    }
    else
    {
      if (y >= 1)
      {
        _STATE = 3;
      }
      else if (y <= -1)
      {
        _STATE = 2;
      }
    }
  }
  else
  {
    int remain = _ORI % 90;
    if (abs(remain) >= 45)
    {
      _STATE = (remain < 0) ? 2 : 3;
      _TURN = 90 - abs(remain);
    }
    else
    {
      _STATE = (remain > 0) ? 2 : 3;
      _TURN = abs(remain);
    }
  }
}

void Robot::udp_init(DynamicJsonDocument &jInfo)
{
  _com.udp_init(_dst, _ID, _POS, &_ORI, jInfo);
}

const char *Robot::get_id()
{
  return _ID;
}

char *Robot::json_creator()
{
  int *POS = get_pos();
  char *msg = _com.json_creator(POS);
  free(POS);
  return msg;
}

int *Robot::get_pos()
{
  int *ptr = (int *)malloc(sizeof(int) * 2);
  for (int i = 0; i < 2; i++)
    ptr[i] = _POS[i];
  return ptr;
}

int Robot::robot_action()
{
  int *dst = _dst.get_dst();
  int *pos = get_pos();
  int arrive = _dst.arrive_dst(pos);
  if (dst[0] != -1 && !arrive)
    decode_action(pos, dst);
  else if (arrive)
    return 1;
  free(dst);
  free(pos);
  return 0;
}

int Robot::get_ori()
{
  return _ORI;
}

int Robot::get_battery()
{
  return _battery;
}

int Robot::get_state()
{
  return _STATE;
}

char *ssid = "nowifi";
char *password = "durf2020";

AsyncUDP _udp;
Robot robot(&encoder1, &encoder2, ssid, password, "5");

void setup()
{
  //  POS = (int*)malloc(sizeof(int) * 2);
  //  dst.dst_init();
  //  loco.motor_init();
  //  com.wifi_init();
  //  MUST BE ADDED TO THE setup()
  robot.robot_init();
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
  //  MUST BE ADDED TO THE setup()
}

void loop()
{
  robot.execute_main();

}
