////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////THE ID OF THE ROBOT NEEDS TO BE SET ACCORDING TO THE ARUCO CODE IT CARRIES/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define PWMA 27
//#define DIRA 14
//#define PWMB 12
//#define DIRB 13
//#define PULSE 7
//SET ACCORDING TO THE ARUCO CODE

#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include "Wire.h" // This library allows you to communicate with I2C devices.
#define PI 3.14159265
#define SDA 21
#define SCL 22

//const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int prev;
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z;                            // variables for gyro raw data
int16_t temperature;                                       // variables for temperature data
char tmp_str[7];                                           // temporary variable used in convert function
char *convert_int16_to_str(int16_t i)
{ // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// code below was copied from http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
// only for reference
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
void Compute()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    errSum += error;
    double dErr = (error - lastErr);

    /*Compute PID Output*/
    Output = kp * error + ki * errSum + kd * dErr;

    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
  }
}

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}
// end copy

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
    int _prev_tick[2];
    int _calc_ori(int ori);
    int _calc_error(int *pos, const char *traj);

  public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);
    //    PID
    //    unsigned long lastTime;
    //    double Input, Output, Setpoint;
    //    double errSum, lastErr;
    //    double kp, ki, kd;
    //    int SampleTime = 1000; //1 sec
    //    PID
    int get_pulse();
    int turn(int deg, float *pos, char dir);
    int forward(int dist, float *pos);
    int *get_tick(int *tick);
    void motor_init();
    void encoder_reset();
    void pid_compute();
    void pid_tuning(double Kp, double Ki, double Kd);
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor1(encoder1, 27, 14, 1),
  _motor2(encoder2, 12, 13, 2)
{
  _PULSE = 7;
  for (int i = 0; i < 2; i++)
    _prev_tick[i] = 0;
}

int Locomotion::forward(int dist, float *_pos)
{
  double spd = 200;
  int tick1 = _motor1.get_tick();
  int tick2 = _motor2.get_tick();
  Serial.println(tick1);
  Serial.print("Position: "); Serial.println(_pos[0]);
  if (tick1 < dist)
  {
    //    see if the error has exceeded certain range
    if (tick1 <= 250 && tick2 <= 250)
    {
      spd = map(tick1, 0, 250, 120, 200);
    }
    else if ((dist - tick1) <= 250)
    {
      spd = (dist - tick1) * 0.8;
      spd = map(spd, 0, 250, 120, 200);
    }
    // the ticks of the motors don't need to be the same
    if (abs(tick1 - tick2) > 5)
    {
      if (tick1 < tick2)
      {
        int error = (tick2 - tick1) * 0.5;
        _motor1.motor_move(spd + error, 0);
        _motor2.motor_move(spd - error, 0);
      }
      else if (tick1 > tick2)
      {
        int error = (tick1 - tick2) * 0.5;
        _motor1.motor_move(spd - error, 0);
        _motor2.motor_move(spd + error, 0);
      }
    }
    _motor1.motor_move(spd, 0);
    _motor2.motor_move(spd, 0);
    // also update its position as well
    int ori = _pos[2];
    //  classify based on the orientation of the robot
    int tick = _motor1.get_tick();
    if (abs(ori <= 5))
      _pos[1] -= tick - _prev_tick[0] ;
    else if (abs(ori) >= 175)
      _pos[1] += tick - _prev_tick[0];
    else if (ori <= 95 && ori >= 85)
      _pos[0] += tick - _prev_tick[0];
    else if (ori <= -85 && ori >= -95)
      _pos[0] -= tick - _prev_tick[0];
    _prev_tick[0] = tick;
    return 0;
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      _motors[i].motor_stop();
      _prev_tick[i] = 0;
    }
    return 1;
  }
}

int Locomotion::turn(int deg, float *_pos, char dir)
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
    int tick = _motor1.get_tick();
    _prev_tick[0] = tick;
    return 0;
  }
  else
  {
    switch (dir)
    {
      case 'L':
        _pos[2] -= deg;
        if (_pos[2] < -180)
          _pos[2] += 360;
        break;
      case 'R':
        _pos[2] += deg;
        if (_pos[2] > 180)
          _pos[2] -= 360;
        break;
    }
    for (int i = 0; i < 2; i++)
    {
      _motors[i].motor_stop();
      _prev_tick[i] = 0;
    }
    Serial.print(_motors[0].get_tick());
    return 1;
    // STATE = 0;
  }
}

int *Locomotion::get_tick(int *tick)
{
  tick[0] = _motor1.get_tick();
  tick[1] = _motor2.get_tick();
  return tick;
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

void Locomotion::encoder_reset()
{
  for (int i = 0; i < 2; i++)
    _motors[i].motor_stop();
}

int Locomotion::get_pulse()
{
  return _PULSE;
}

class Robot
{
  private:
    int _ptr;
    int _turn;
    int _dist;
    int _error;
    int _STATE;
    int _width;
    int _task_size;
    int _off_course;
    int _prev_tick[2];
    int **_route;
    float _pos[3]; // this contains the current coordinate as well as the rotation of the robot [x, y, u]
    Locomotion _loc;

  public:
    Robot(Encoder *encoder1, Encoder *encoder2);
    int *get_pos();
    int get_state();
    void robot_init();
    void calc_error();
    void check_task();
    void update_est();
    void main_executor();
    void action_decoder();
    void update_abs(int *pos);
    void auto_route(int *dst);
};

Robot::Robot(Encoder *encoder1, Encoder *encoder2) : _loc(encoder1, encoder2)
{
  _ptr = 1;
  _STATE = 0;
  _width = 8;
  _off_course = 0;
}

int *Robot::get_pos()
{
  int *ptr = (int *)malloc(sizeof(int) * 3);
  for (int i = 0; i < 3; i++)
    ptr[i] = _pos[i];
  return ptr;
}

int Robot::get_state()
{
  return _STATE;
}

void Robot::robot_init()
{
  for (int i = 0; i < 2; i++)
    _pos[i] = 0;
  _loc.motor_init();
  _pos[2] = 0;
}

void Robot::calc_error()
{
  int error;
  int *prev_task = _route[_ptr - 1];
  int *crt_task = _route[_ptr];
  if (!(crt_task[0] - prev_task[0]))
  {
    if (crt_task[1] - prev_task[1] < 0) // from south to north
      error = _pos[0] - crt_task[0];
    else
      error = crt_task[0] - _pos[0];
  }
  else if (!(crt_task[1] - prev_task[1]))
  {
    if (crt_task[0] - prev_task[0] < 0) // from west to east
      error = _pos[1] - crt_task[1];
    else
      error = crt_task[1] - _pos[1];
  }
  if (abs(error) > 3) // a threshold value needs to be set
    // change the state to error recovery
    _off_course = 1;
}

void Robot::auto_route(int *dst)
{
  // A* routing algorithm goes here
  int inter = 1;
  _route = (int **)malloc(sizeof(int *) * (inter + 1));
  for (int i = 0; i < (inter + 1); i++)
    _route[i] = (int *)malloc(sizeof(int) * 2);
  //  hard-coding the current position and the destination
  for (int i = 0; i < 2; i++)
    _route[0][i] = _pos[i];
  _route[1][0] = 2000;
  _route[1][1] = 0;
  _task_size = inter + 1;
}

// this function would be called when the encoder ticks or when the server updates the robots global position
void Robot::update_abs(int *pos)
{
  for (int i = 0; i < 3; i++)
    _pos[i] = pos[i];
  free(pos);
}

//void Robot::update_est()
//{
//  //  formulas for calculating trajectory based on encoders' ticks
//  int encoder_tick[2];
//  _loc.get_tick(encoder_tick);
//  //  code below is for calculating estimated current position
//  int left = encoder_tick[0];
//  int right = encoder_tick[1];
//  int error;
//  int pulse = _loc.get_pulse();
//  //  if (abs(left - right) > 10 && !_off_course)
//  //  {
//  //    left *= 0.01;
//  //    right *= 0.01;
//  //    _off_course = 1;
//  //    int rotation = (right - left) / _width;
//  //    int radius = _width * (left + right) / 2 * (right - left);
//  //    if (left > right)
//  //    {
//  //      // right rotation
//  //      _pos[0] = radius * sin((_pos[2] + rotation) * PI / 180) + _pos[0] - radius * sin(_pos[2]);
//  //      _pos[1] = -radius * cos((_pos[2] + rotation) * PI / 180) + _pos[1] + radius * cos(_pos[2]);
//  //      _pos[2] += rotation;
//  //      if (_pos[2] > 180)
//  //        _pos[2] -= 360;
//  //    }
//  //    else
//  //    {
//  //      // left rotation
//  //      _pos[0] = radius * sin((_pos[2] - rotation) * PI / 180) + _pos[0] - radius * sin(_pos[2]);
//  //      _pos[1] = -radius * cos((_pos[2] - rotation) * PI / 180) + _pos[1] + radius * cos(_pos[2]);
//  //      _pos[2] -= rotation;
//  //      if (_pos[2] < -180)
//  //        _pos[2] += 360;
//  //    }
//  //  }
//  if (_off_course)
//  {
//    int ori = _pos[2];
//    int r;
//    // need to know how much it went off course
//    if (error > 0)
//    {
//      r = left * 4 / (right - left);
//    }
//  }
//  else
//  {
//    switch (_STATE)
//    {
//    case 1:
//      //  when the robot is going forward
//      {
//        int ori = _pos[2];
//        //  classify based on the orientation of the robot
//        int diff = (left + right - _prev_tick[0] - _prev_tick[1]);
//        if (abs(ori <= 5))
//          _pos[1] -= diff / 2;
//        else if (abs(ori) >= 175)
//          _pos[1] += diff / 2;
//        else if (ori <= 95 && ori >= 85)
//          _pos[0] += diff / 2;
//        else if (ori <= -85 && ori >= -95)
//          _pos[0] -= diff / 2;
//      }
//      break;
//    case 2:
//      if (left >= _turn * pulse - 2)
//      {
//        _pos[2] -= _turn;
//        if (_pos[2] < -180)
//          _pos[2] += 360;
//      }
//      break;
//    case 3:
//      if (left >= _turn * pulse - 2)
//      {
//        _pos[2] += _turn;
//        if (_pos[2] > 180)
//          _pos[2] -= 360;
//      }
//      break;
//    }
//  }
//  //  knowing how far it has gone so far
//  for (int i = 0; i < 2; i++)
//    _prev_tick[i] = encoder_tick[i];
//}

void Robot::main_executor()
{
  int proceed = 0;
  switch (_STATE)
  {
    case 0:
      check_task();
      if (_STATE != 4) action_decoder();
      break;
    case 1:
      proceed = _loc.forward(_dist, _pos);
      Serial.println("Moving forward");
      break;
    case 2:
      proceed = _loc.turn(_turn, _pos, 'L');
      Serial.println("turning left");
      break;
    case 3:
      proceed = _loc.turn(_turn, _pos, 'R');
      Serial.println("turning right");
      break;
    case 4:
      Serial.println("arrived");
      break;
  }
  if (proceed)
    _STATE = 0;
  if (!_STATE) Serial.println("resetting");
  Serial.print(_pos[0]); Serial.print(" "); Serial.println(_pos[1]);
  // update_est();
  Serial.println();
}

void Robot::action_decoder()
{
  int x = _route[_ptr][0] - _pos[0];
  int y = _route[_ptr][1] - _pos[1];
  int ORI = _pos[2];
  _turn = 90;
  if (abs(ORI) <= 5)
  {
    if (y >= 1)
    {
      _STATE = 3;
      _turn = 180;
    }
    else if (y <= -1)
    {
      _STATE = 1;
      _dist = y;
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
  else if (abs(ORI) >= 175)
  {
    if (y >= 1)
    {
      _STATE = 1;
      _dist = abs(y);
    }
    else if (y <= -1)
    {
      _STATE = 3;
      _turn = 180;
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
  else if (ORI >= 85 && ORI <= 95)
  {
    if (x >= 1)
    {
      _STATE = 1;
      _dist = x;
    }
    else if (x <= 1)
    {
      _STATE = 3;
      _turn = 180;
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
  else if (ORI >= -95 && ORI <= -85)
  {
    if (x >= 1)
    {
      _STATE = 3;
      _turn = 180;
    }
    else if (x <= -1)
    {
      _STATE = 1;
      _dist = abs(x);
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
    int remain = ORI % 90;
    if (abs(remain) >= 45)
    {
      _STATE = (remain < 0) ? 2 : 3;
      _turn = 90 - abs(remain);
    }
    else
    {
      _STATE = (remain > 0) ? 2 : 3;
      _turn = abs(remain);
    }
  }
}

void Robot::check_task()
{
  int *pos = get_pos();
  Serial.println("Checking available tasks");
  if (abs(pos[0] - _route[_ptr][0]) <= 5 && abs(pos[1] == _route[_ptr][1]) <= 5)
  {
    Serial.println("arrived at some dst");
    if (_ptr + 1 == _task_size)
    {
      //  robot arriving at the final destination
      Serial.println("arrived");
      _STATE = 4;
      for (int i = 0; i < _task_size; i++)
        free(_route[i]);
      //        clear every node in array
      free(_route);
      _task_size = 0;
      //      _loc.reset_encoder();
      _ptr = 1;
      _turn = 0;
      _dist = 0;
      _error = 0;
    }
    // not arrived at the final destination
    else
    {
      //  robot execute the next task
      _ptr++;
    }
  }
  free(pos);
}

Robot robot(&encoder1, &encoder2);
//Locomotion loc(&encoder1, &encoder2);
AsyncUDP udp;
//
const char *ssid = "nowifi";
const char *password = "durf2020";

void setup()
{
  //  SET UP THE ID OF THE ROBOT HERE
  //  loc.motor_init();
  robot.robot_init();
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
  //  WiFi.mode(WIFI_STA);
  //  WiFi.begin(ssid, password);
  Serial.begin(115200);
  //  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  //  {
  //    Serial.println("WiFi Failed");
  //    while (1)
  //    {
  //      delay(1000);
  //    }
  //  }
  //  if (udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001))
  //  {
  //    udp.onPacket([](AsyncUDPPacket packet) {});
  //  }
  delay(1000);
  //  Wire.begin(SDA, SCL, 400000);
  //  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  //  Wire.write(0x6B); // PWR_MGMT_1 register
  //  Wire.write(0); // set to zero (wakes up the MPU-6050)
  //  Wire.endTransmission(true);
  //  prev = millis();
  //  listening to both task and current position on this channel
  int *dst;
  robot.auto_route(dst);
  Serial.println("initialized");
}

void loop()
{
  robot.main_executor();
  //  if (!STATE)
  //    STATE = loc.forward(10000);
}
