////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////THE ID OF THE ROBOT NEEDS TO BE SET ACCORDING TO THE ARUCO CODE IT CARRIES/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#define PWMA 27
//#define DIRA 14
//#define PWMB 12
//#define DIRB 13
//#define PULSE 7
//SET ACCORDING TO THE ARUCO CODE

int DIST;
int STATE = 0;
int PID = 0.2;

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
  int forward(int dist, int *pos, const char *path);
  int turn(int deg, char dir);
  int *get_tick();
  void motor_init();
  void encoder_reset();
  void pid_compute();
  void pid_tuning(double Kp, double Ki, double Kd);
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor1(encoder1, 27, 14, 1),
                                                               _motor2(encoder2, 12, 13, 2)
{
  _PULSE = 7;
}

int Locomotion::_calc_error(int *pos, const char *traj)
{
  int error;
  int path = traj[1].toInt();
  switch (traj[0])
  {
  case 'x':
    if (abs(pos[2]) <= 10)
      error = pos[0] - path;
    else
      error = path - pos[0];
    break;
  case 'y':
    if (pos[2] >= 80 && pos[2] <= 100)
      error = pos[1] - path;
    else
      error = path - pos[1];
    break;
  }
  return error;
}

int Locomotion::_calc_ori(int ori, const char *traj)
{
  char dir = traj[0];
  if (dir == 'y')
  {
    if (ori > 0)
    {
      ori = ori - 90;
    }
    else
    {
      ori = ori + 90;
    }
  }
  else
  {
    if (abs(ori) > 175)
    {
      if (ori > 0)
      {
        ori -= 180;
      }
      else
      {
        ori += 180;
      }
    }
  }
  return ori;
}

int Locomotion::forward(int dist, int *pos, const char *traj)
{
  double spd = 200;
  int tick1 = _motor1.get_tick();
  int tick2 = _motor2.get_tick();
  if (tick1 < dist)
  {
    //    compare the current position with the position in camera
    //    error is the difference in the prediction position and the position from the camera
    int ori = _calc_ori(pos[2], traj);
    int error = _calc_error(pos, traj);
    //    error is exceeding 3 cm
    if (abs(error) >= 3)
    {
      //  need to determine the rotation relative to the current trajectory
      int ori = _calc_ori(pos[2], traj);
      //  use triangulation to calculate the speed for the motor
    }
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
    else
    {
      _motor1.motor_move(spd, 0);
      _motor2.motor_move(spd, 0);
    }
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

int *Locomotion::get_tick()
{
  int *ptr = (int *)malloc(sizeof(int) * 2);
  ptr[1] = _motor1.get_tick();
  ptr[2] = _motor2.get_tick();
  return ptr;
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
  pid_tuning(0.5, 0.5, 0.5);
}

void Locomotion::encoder_reset()
{
  for (int i = 0; i < 2; i++)
    _motors[i].motor_stop();
}

void Locomotion::pid_compute()
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

void Locomotion::pid_tuning(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

class Robot
{
private:
  int _ptr;
  int _turn;
  int _dist;
  int _error;
  int _STATE;
  int _task_size;
  int _off_course;
  int *_prev_tick;
  int **_route;
  int _pos[3]; // this contains the current coordinate as well as the rotation of the robot [x, y, u]
  Locomotion _loc;

public:
  Robot(Encoder *encoder1, Encoder *encoder2);
  int *get_pos();
  void robot_init();
  void calc_error();
  void check_task();
  void update_est();
  void update_abs(int *pos);
  void auto_route(int *dst);
};

Robot::Robot(Encoder *encoder1, Encoder *encoder2) : _loc(encoder1, encoder2)
{
  _ptr = 1;
  _STATE = 0;
}

int *Robot::get_pos()
{
  //  free needed
  int *ptr = (int *)malloc(sizeof(int) * 3);
  for (int i = 0; i < 3; i++)
    ptr[i] = _pos[i];
  return ptr;
}

void Robot::robot_init()
{
  //  routing algorithm goes here
  //  [x, y, rotation]
  for (int i = 0; i < 3; i++)
    _pos[i] = 0;
  for (int i = 0; i < 2; i++)
    _prev_tick[i] = 0;
}

void Robot::calc_error()
{
  //  first calculate the path currently on
  int *prev_task = _route[_ptr - 1];
  int *crt_task = _route[_ptr];
  if (!(crt_task[0] - prev_task[0]))
  {
    
  }
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
  _route[1][0] = 20;
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

void Robot::update_est()
{
  //  formulas for calculating trajectory based on encoders' ticks
  int *tick = _loc.get_tick();
  //  code below is for calculating estimated current position
  int left = tick[0] - _prev_tick[0];
  int right = tick[1] - _prev_tick[1];
  //  knowing how far it has gone so far
  switch (_STATE)
  {
  case 1:
    //  when the robot is going forward
    {
      int ori = _pos[2];
      //  classify based on the orientation of the robot
      if (abs(ori <= 5))
        _pos[1] -= 0.01 * (left + right) / 2;
      else if (abs(ori) >= 175)
        _pos[1] += 0.01 * (left + right) / 2;
      else if (ori <= 95 && ori >= 85)
        _pos[0] += 0.01 * (left + right) / 2;
      else if (ori <= -85 && ori >= -95)
        _pos[0] -= 0.01 * (left + right) / 2;
    }
    break;
  case 2:
    // when the robot is turning
    // not filled for now
    break;
  }
  _prev_tick[0] = tick[0];
  _prev_tick[1] = tick[1];
  free(tick);
}

void Robot::check_task()
{
  int *pos = get_pos();
  if (pos[0] == _route[_ptr][0] && pos[1] == _route[_ptr][1])
  {
    if (_ptr + 1 == _task_size)
    {
      Serial.println("arrived");
      _STATE = 0;
      for (int i = 0; i < _task_size; i++)
        free(_route[i]);
      //        clear every node in array
      free(_route);
      _task_size = 0;
      _loc.reset_encoder();
      _ptr = 1;
      for (int i = 0; i < 2; i++)
        _prev_tick[i] = 0;
      _turn = 0;
      _dist = 0;
      _error = 0;
    }
    // not arrived at the final destination
    else
    {
      _ptr++;
    }
  }
}

Robot robot(&encoder1, &encoder2);

void setup()
{
  //  SET UP THE ID OF THE ROBOT HERE
  loc.motor_init();
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;

  Serial.begin(115200);
  delay(1000);
  //  listening to both task and current position on this channel
}

void loop()
{
  //  if (!STATE)
  //    STATE = loc.forward(10000);
}
