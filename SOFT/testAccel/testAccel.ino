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
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define PI 3.14159265

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 22      // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SDA 21
#define SCL 22

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int prev;
int STATE = 0;

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

MPU6050 mpu;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

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
    int pos[3];
    int forward(int dist);
    int turn(int deg, char dir);
    int *get_tick(int *tick);
    void motor_init();
    void pid_compute();
    void encoder_reset();
    void accel_update_pos(int accel);
    void pid_tuning(double Kp, double Ki, double Kd);
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor1(encoder1, 27, 14, 1),
  _motor2(encoder2, 12, 13, 2)
{
  _PULSE = 7;
  for (int i = 0; i < 2; i++)
    _prev_tick[i] = 0;
}

int Locomotion::forward(int dist)
{
  double spd = 200;
  int tick1 = _motor1.get_tick();
  int tick2 = _motor2.get_tick();
  if (tick1 < dist)
  {
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
    _motor1.motor_move(spd, 0);
    _motor2.motor_move(spd, 0);
    int ori = pos[2];
    int tick = _motor1.get_tick();
    if (abs(ori <= 5))
      pos[1] -= tick - _prev_tick[0];
    else if (abs(ori) >= 175)
      pos[1] += tick - _prev_tick[0];
    else if (ori <= 95 && ori >= 85)
      pos[0] += tick - _prev_tick[0];
    else if (ori <= -85 && ori >= -95)
      pos[0] -= tick - _prev_tick[0];
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
    int tick = _motor1.get_tick();
    _prev_tick[0] = tick;
    return 0;
  }
  else
  {
    switch (dir)
    {
      case 'L':
        pos[2] -= deg;
        if (pos[2] < -180)
          pos[2] += 360;
        break;
      case 'R':
        pos[2] += deg;
        if (pos[2] > 180)
          pos[2] -= 360;
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

void Locomotion::accel_update_pos(int accel)
{
}

// Robot robot(&encoder1, &encoder2);
Locomotion loc(&encoder1, &encoder2);
AsyncUDP udp;

const char *ssid = "nowifi";
const char *password = "durf2020";

void setup()
{
  //  SET UP THE ID OF THE ROBOT HERE
  loc.motor_init();
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.begin(115200);
  Wire.begin(SDA, SCL, 400000);
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
    while (1)
    {
      delay(1000);
    }
  }
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(3000);
  prev = millis();
}

void loop()
{
  Serial.println("imu ready");
  int current = millis();
  if (!STATE)
    STATE = loc.forward(10000);
  if (current - prev >= 500)
  {
    if (!dmpReady)
      return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
      char jsonStr[80];
      sprintf(jsonStr, " %.2f", aaWorld.x);
      udp.broadcast(jsonStr);
    }
    prev = current;
  }
}
