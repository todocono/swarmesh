#include "Robot.h"
#include "Arduino.h"

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