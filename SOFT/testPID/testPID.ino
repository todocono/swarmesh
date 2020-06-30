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
  public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);

    //    PID
    unsigned long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;
    int SampleTime = 1000; //1 sec
    //    PID

    int turn(int deg, char dir);
    int forward(int dist);
    void motor_init();
    void pid_compute();
    void pid_tuning(double Kp, double Ki, double Kd);
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor1(encoder1, 27, 14, 1),
  _motor2(encoder2, 12, 13, 2)
{
  _PULSE = 7;
}

int Locomotion::forward(int dist)
{
  if (_motor1.get_tick() < dist)
  {
    //    define a function revealing the current position
    //    compare the current position with the position in camera
    //    error is the difference in the prediction position and the position from the camera

    double spd = 255;
    int tick = _motor1.get_tick();
    if (tick <= 250)
    {
      spd = map(tick, 0, 250, 150, 255);
      Serial.println(spd);
    }
    else if ((dist - tick) <= 250)
    {
      Serial.println("stopping");
      spd = (dist - tick) * 0.8;
      spd = map(spd, 0, 250, 150, 255);
    }
    if (_motor1.get_tick() < _motor2.get_tick())
    {
      //          PID code here
      int error = _motor2.get_tick() - _motor1.get_tick();

      _motor1.motor_move(spd, 0);
      _motor2.motor_move(spd / error, 0);
    }
    else if (_motor1.get_tick() > _motor2.get_tick())
    {
      int error = _motor1.get_tick() - _motor2.get_tick();

      _motor1.motor_move(spd / error, 0);
      _motor2.motor_move(spd, 0);
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
    int ptr;
    int *_pos;
    int **_route;
  public:
    Robot();
    void robot_init();
    void get_prev_pos();
};

Robot::Robot()
{
  ptr = 1;
}

void Robot::robot_init()
{
  _pos = (int*)malloc(sizeof(int) * 2);
  _route = (int**)malloc(sizeof(int*) * 2);
  for (int i = 0; i < 2; i ++) _route[i] = (int*)malloc(sizeof(int) * 2);
  _pos[0] = 0;
  _pos[1] = 0;
  //  hard-coding the current position and the route
  for (int i = 0; i < 2; i ++) _route[0][i] = _pos[i];
  _route[1][0] = 20;
}

//void Robot::get_prev_pos()
//{
//  int *ptr = (int*)malloc(sizeof(int) * 2);
//  for (int i = 0; i < 2; i ++) ptr[i] = _route[ptr - 1][i];
//  return ptr;
//}

Locomotion loc(&encoder1, &encoder2);
Robot robot;

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
  if (!STATE)
    STATE = loc.forward(2000);
}
