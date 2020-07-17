////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////THE ID OF THE ROBOT NEEDS TO BE SET ACCORDING TO THE ARUCO CODE IT CARRIES/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

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

public:
    Motor(Encoder *encoder, int PWM, int DIR, int IDX);
    void motor_move(int spd, int dir);
    void motor_stop();
    void reset_encoder();
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

void Motor::reset_encoder()
{
    (*_encoder).numberTicks = 0;
}

void Motor::motor_stop()
{
    reset_encoder();
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

class PID
{
private:
    unsigned long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;
    int SampleTime;

public:
    PID();
    int pid_compute(int error);
    void pid_tuning(double Kp, double Ki, double Kd);
};

PID::PID()
{
    kp = 0.005;
    ki = 0.0;
    kd = 0.0;
    errSum = 0;
    lastErr = 0;
    SampleTime = 1000;
}

int PID::pid_compute(int error)
{
    Output = kp * error; // + ki * errSum + kd * dErr;
    // unsigned long now = millis();
    // int timeChange = (now - lastTime);
    // if (timeChange >= SampleTime)
    // {
    //     /*Compute all the working error variables*/
    //     errSum += error;
    //     double dErr = (error - lastErr);

    //     /*Compute PID Output*/
    //     Output = kp * error + ki * errSum + kd * dErr;

    //     /*Remember some variables for next time*/
    //     lastErr = error;
    //     lastTime = now;
    // }
    return Output;
}

void PID::pid_tuning(double Kp, double Ki, double Kd)
{
    double SampleTimeInSec = ((double)SampleTime) / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

class Locomotion
{
private:
    PID _pid;
    Motor _motor1;
    Motor _motor2;
    Motor *_motors;
    int _PULSE;
    int _width;
    int _off_course;
    int _prev_tick[2];
    int _calc_ori(int ori);
    int _calc_error(int *pos, const char *traj);

public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);
    float pos[3];
    int get_pulse();
    int forward(int dist, int error);
    int turn(int deg, char dir);
    void tune_pid(double kp);
    void reset_encoders();
    void motor_init();
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor2(encoder1, 27, 14, 1),
                                                               _motor1(encoder2, 12, 13, 2)
{
    _PULSE = 7;
    _width = 800;
    _off_course = 0;
    for (int i = 0; i < 2; i++)
        _prev_tick[i] = 0;
}

void Locomotion::tune_pid(double kp)
{
    _pid.pid_tuning(kp, 0, 0);
}

int Locomotion::forward(int dist, int error)
{
    int spd = 200;
    if (abs(error) >= 100)
        _off_course = 1;
    else
        _off_course = 0;
    int tick1 = _motor1.get_tick();
    int tick2 = _motor2.get_tick();
    if (tick1 < dist)
    {
        int gap = 0;
        if (tick1 <= 250 && tick2 <= 250)
            spd = map(tick1, 0, 250, 120, 200);
        else if ((dist - tick1) <= 250)
            spd = map((dist - tick1) * 0.8, 0, 250, 120, 200);
        if (!_off_course)
        {
            if (abs(tick1 - tick2) > 5)
                gap = (tick1 - tick2) * 0.5;
            _motor1.motor_move(spd - gap, 0);
            _motor2.motor_move(spd + gap, 0);
        }
        else
        {
            int gap = _pid.pid_compute(error);
            Serial.println(gap);
            _motor1.motor_move(spd - gap, 0);
            _motor2.motor_move(spd + gap, 0);
        }
        int tick1 = _motor1.get_tick();
        int tick2 = _motor2.get_tick();
        int ori = pos[2];
        if (abs(ori) <= 5)
            pos[1] -= tick1 - _prev_tick[0];
        else if (abs(ori) >= 175)
            pos[1] += tick1 - _prev_tick[0];
        else if (ori <= 95 && ori >= 85)
            pos[0] += tick1 - _prev_tick[0];
        else if (ori <= -85 && ori >= -95)
            pos[0] -= tick1 - _prev_tick[0];
        _prev_tick[0] = tick1;
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
        num1 = 1;
        num2 = 0;
        break;

    case 'R':
        num1 = 0;
        num2 = 1;
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
        return 1;
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

void Locomotion::reset_encoders()
{
    for (int i = 0; i < 2; i++)
    {
        _motors[i].reset_encoder();
        _prev_tick[i] = 0;
    }
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
    int _dst[2];
    float *_pos; // this contains the current coordinate as well as the rotation of the robot [x, y, u]
    int **_route;
    char _direction;
    const char *_ID = "2";
    Locomotion _loc;

public:
    Robot(Encoder *encoder1, Encoder *encoder2);
    const char *get_id();
    int get_size();
    int get_state();
    int get_error();
    int *get_pos();
    int **get_route();
    void robot_init();
    void calc_error();
    void check_task();
    void auto_route();
    void update_est();
    void main_executor();
    void action_decoder();
    void reroute(char dir);
    void tune_pid(double kp);
    void update_abs(int *pos);
    void update_error(int *pos);
    void update_state(int state);
};

Robot::Robot(Encoder *encoder1, Encoder *encoder2) : _loc(encoder1, encoder2)
{
    _ptr = 1;
    _STATE = 0;
    _width = 8;
    _off_course = 0;
    _pos = _loc.pos;
}

int Robot::get_error()
{
    return _error;
}

int *Robot::get_pos()
{
    int *ptr = (int *)malloc(sizeof(int) * 3);
    for (int i = 0; i < 3; i++)
        ptr[i] = _pos[i];
    return ptr;
}

const char *Robot::get_id()
{
    return _ID;
}

int Robot::get_size()
{
    return _task_size;
}

int Robot::get_state()
{
    return _STATE;
}

void Robot::robot_init()
{
    _loc.motor_init();
}

void Robot::calc_error()
{
    int *pos = get_pos();
    int *crt_task = _route[_ptr];
    float error1 = pos[0] - crt_task[0];
    float error2 = pos[1] - crt_task[1];
    switch (_direction)
    {
    case 'N':
        _error = error1;
        break;
    case 'S':
        _error = -error1;
        break;
    case 'W':
        _error = -error2;
        break;
    case 'E':
        _error = error2;
        break;
    }
    free(pos);
}

void Robot::auto_route()
{
    // A* routing algorithm goes here
    float dst[2];
    dst[0] = 1500;
    dst[1] = 15000;
    int x_distance = (dst[0] - _pos[0]) / 1500;
    int y_distance = (dst[1] - _pos[1]) / 1500;

    // randomly decide where to break the route
    if (x_distance && y_distance)
    {
        Serial.println("setting break points");
        // when both x_distance and y_distance is greater than 0
        int random_num = rand() % abs(x_distance);
        _route = (int **)malloc(sizeof(int *) * 4);
        for (int i = 0; i < 4; i++)
            _route[i] = (int *)malloc(sizeof(int) * 2);
        //
        for (int i = 0; i < 2; i++)
            _route[0][i] = _pos[i];
        _task_size = 4;
        if (x_distance > 0)
            _route[1][0] = _pos[0] + random_num * 1500;
        else
            _route[1][0] = _pos[0] - random_num * 1500;
        _route[1][1] = _pos[1];
        _route[2][0] = _route[1][0];
        _route[2][1] = _route[1][1] + y_distance * 1500;
        _route[3][0] = dst[0];
        _route[3][1] = dst[1];
    }
    else
    {
        _task_size = 2;
        _route = (int **)malloc(sizeof(int *) * _task_size);
        for (int i = 0; i < 2; i++)
            _route[i] = (int *)malloc(sizeof(int) * 2);
        for (int i = 0; i < 2; i++)
        {
            _route[0][i] = _pos[i];
            _route[1][i] = dst[i];
        }
    }
}

void Robot::tune_pid(double kp)
{
    _loc.tune_pid(kp);
}

void Robot::update_state(int state)
{
    _STATE = state;
}

void Robot::update_abs(int *pos)
{
    // calculate the difference between the estimate and the actual
    // compensate for the difference using the _dist variable
    if (_STATE == 1)
    {
        _loc.reset_encoders();
        if (_direction == 'N' || _direction == 'S')
            _dist = abs(_route[_ptr][1] - pos[1]);
        else
            _dist = abs(_route[_ptr][0] - pos[0]);
    }
    for (int i = 0; i < 3; i++)
        _pos[i] = pos[i];
}

void Robot::update_error(int *pos)
{
    int *crt_task = _route[_ptr];
    int error1 = pos[0] - crt_task[0];
    int error2 = pos[1] - crt_task[1];
    switch (_direction)
    {
    case 'N':
        _error = error1;
        break;
    case 'S':
        _error = -error1;
        break;
    case 'W':
        _error = -error2;
        break;
    case 'E':
        _error = error2;
        break;
    }
}

int **Robot::get_route()
{
    return _route;
}

void Robot::main_executor()
{
    int proceed = 0;
    switch (_STATE)
    {
    case 0:
        check_task();
        if (_STATE != 4)
            action_decoder();
        break;
    case 1:
        proceed = _loc.forward(_dist, _error);
        //      Serial.println("Moving forward");
        break;
    case 2:
        proceed = _loc.turn(_turn, 'L');
        //      Serial.println("turning left");
        break;
    case 3:
        proceed = _loc.turn(_turn, 'R');
        //      Serial.println("turning right");
        break;
    case 4:
        //      Serial.println("arrived");

        break;
    }
    if (proceed)
        _STATE = 0;
    // update_est();
}

/*
  same as the previous action decoder
  take in the upcoming destination in _route
  output the necessary action to get to the destination
*/

void Robot::action_decoder()
{
    int *pos = get_pos();
    int x = _route[_ptr][0] - pos[0];
    int y = _route[_ptr][1] - pos[1];
    free(pos);
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
            _direction = 'N';
            _dist = abs(y);
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
            _direction = 'S';
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
            _direction = 'E';
            _dist = abs(x);
        }
        else if (x <= -1)
        {
            _STATE = 3;
            _turn = 180;
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
            _direction = 'W';
            _dist = abs(x);
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
    Serial.print(_pos[0]);
    Serial.print("  ");
    Serial.print(_pos[1]);
    Serial.print("  ");
    Serial.println(_pos[2]);
    if (abs(pos[0] - _route[_ptr][0]) <= 50 && abs(pos[1] == _route[_ptr][1]) <= 50)
    {
        Serial.println("arrived at a dst");
        if (_ptr + 1 == _task_size)
        {
            //  robot arriving at the final destination
            _STATE = 4;
            for (int i = 0; i < _task_size; i++)
                free(_route[i]);
            free(_route);
            _task_size = 0;
            _ptr = 1;
            _turn = 0;
            _dist = 0;
            _error = 0;
        }
        else
        {
            _ptr++;
        }
    }
    free(pos);
}

Robot robot(&encoder1, &encoder2);
AsyncUDP udp;

const char *ssid = "nowifi";
const char *password = "durf2020";

int ctr = 0;

void setup()
{
    //  SET UP THE ID OF THE ROBOT HERE
    robot.robot_init();
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.begin(115200);
    robot.tune_pid(0.003);
    robot.update_state(-1);
    Serial.println("initializing robot");
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
            {
                const char *ID;
                ID = "7";
                if (!jInfo[ID][0][0] && !jInfo[ID][0][1])
                    break;
                int pos_n[3];
                for (int i = 0; i < 2; i++)
                    pos_n[i] = jInfo[ID][0][i];
                pos_n[2] = jInfo[ID][1];
                robot.update_abs(pos_n);
                if (robot.get_state() == 1)
                  robot.calc_error();
                char jsonStr[80];
                //              jsonCreator(jsonStr);
                const size_t capacity = JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(2);
                DynamicJsonDocument doc(capacity);
                doc["Error"] = robot.get_error();
                // JsonArray Pos = doc.createNestedArray("Pos");
                // Pos.add(POS[0]);
                // Pos.add(POS[1]);
                serializeJson(doc, jsonStr);
                doc.clear();
                udp.broadcast(jsonStr);
                if (!ctr)
                {
                    robot.update_state(0);
                    robot.auto_route();
                    ctr++;
                }
                break;
            }
            case 2:
            {
                int pos_now[3] = {22500, 15000, -90};
                robot.update_state(0);
                robot.update_abs(pos_now);
                robot.auto_route();
                break;
            }
            case 3:
            {
                double kp = jInfo["PID"];
                robot.tune_pid(kp);
                udp.broadcast("PID tuned");
                break;
            }
            }
        });
    }
    // robot.update_state(4);
    // float *pos_now = robot.get_pos();
    // Serial.print(pos_now[0]);
    // Serial.print("  ");
    // Serial.println(pos_now[1]);
    delay(1000);
    //  listening to both task and current position on this channel
    Serial.println("initialized");
    // int **route = robot.get_route();
    // for (int i = 0; i < robot.get_size(); i++)
    // {
    //     Serial.print(route[i][0]);
    //     Serial.print("  ");
    //     Serial.println(route[i][1]);
    // }
}

void loop()
{
    if (robot.get_state() != -1)
        robot.main_executor();
}
