// #include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <cmath>
#include <time.h>
#include <cstring>
#include <ctime>
#include <cstdlib>

using namespace std;
using namespace webots;

// Set speed and some const value
static const double maxSpeed = 3;
const double pi = 3.1415926;
bool start = false;
bool arrived = false;
int dst[2];
const int scale = 10000;


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
    float pid_compute(int error);
    void pid_tuning(double Kp, double Ki, double Kd);
};

PID::PID()
{
    kp = 0.0013;
    ki = 0.0;
    kd = 0.0;
    errSum = 0;
    lastErr = 0;
    SampleTime = 1000;
}

float PID::pid_compute(int error)
{
    // error = (error > 5) ? error : 0;
    // cout << "error: " << error << endl;
    Output = kp * error; // + ki * errSum + kd * dErr;
    // cout << "error: " << (float) error/scale << endl;
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



class Slave : public Robot {
public:
    Slave();
    int* get_pos();
    int get_error();
    int get_size();
    int get_state();
    int** get_route();
    void calc_error();
    void check_task();
    void update_est();
    void auto_route();
    void main_executor();
    void action_decoder();
    void reroute(char dir);
    void tune_pid(double kp);
    void update_abs(int* pos);
    void update_error(int* pos);
    void update_state(int state);

    int forward(int _dist, char dir, int error);
    int turn(int deg, char dir);

    void setup();

private:
    PID _pid;
    int _ptr;
    int _turn;
    int _dist;
    int _error;
    int _STATE;
    int _width;
    int _task_size;
    int _off_course;
    int _dst[2];
    int _prev_tick[2];
    int** _route;
    int* _pos; // this contains the current coordinate as well as the rotation of the robot [x, y, u]
    char _direction;

    bool is_moving = false;
    int prev_x;
    int prev_z;
    float prev_ori;
    char forward_dir;

    static double boundSpeed(double speed);

    int timeStep;

    Receiver* receiver;
    // DistanceSensor* distanceSensors[2];
    Motor* motors[2];
    GPS* gp;
    Compass* cp;
    Emitter* emitter;
};

Slave::Slave() {
    timeStep = 32;

    _ptr = 1;
    _STATE = 0;
    _width = 8;
    _off_course = 0;
    _pos = (int*)malloc(sizeof(int) * 3);


    gp = getGPS("global");
    gp->enable(timeStep);

    cp = getCompass("cp");
    cp->enable(timeStep);

    emitter = getEmitter("emitter");

    receiver = getReceiver("receiver");
    receiver->enable(timeStep);
    motors[0] = getMotor("left wheel motor");
    motors[1] = getMotor("right wheel motor");
    motors[0]->setPosition(std::numeric_limits<double>::infinity());
    motors[1]->setPosition(std::numeric_limits<double>::infinity());
    motors[0]->setVelocity(0.0);
    motors[1]->setVelocity(0.0);

    // string distanceSensorNames("ds0");
    // for (int i = 0; i < 2; i++) {
        // distanceSensors[i] = getDistanceSensor(distanceSensorNames);
        // distanceSensors[i]->enable(timeStep);
        // distanceSensorNames[2]++;  // for getting "ds1","ds2",...
    // }
}

double Slave::boundSpeed(double speed) {
    return std::min(maxSpeed, std::max(-maxSpeed, speed));
}

int Slave::get_error()
{
    return _error;
}

int* Slave::get_pos()
{
    const double* north = cp->getValues();
    float rad = atan2(north[0], north[2]);
    float bearing = (rad + 2 * 1.5708) / M_PI * 180.0;
    _pos[0] = gp->getValues()[0] * scale;
    _pos[1] = gp->getValues()[2] * scale;
    _pos[2] = bearing;
    int* ptr = (int*)malloc(sizeof(int) * 3);
    ptr[0] = gp->getValues()[0] * scale;
    ptr[1] = gp->getValues()[2] * scale;
    ptr[2] = bearing;
    return ptr;
}

int Slave::get_size()
{
    return _task_size;
}

int Slave::get_state()
{
    return _STATE;
}

void Slave::calc_error()
{
    int* pos = get_pos();
    int* crt_task = _route[_ptr];
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

void Slave::auto_route()
{
    // A* routing algorithm goes here
    srand(time(0));
    int numX = 1 + rand() % 28 - 14;  // 30 grids in x-axis
    int numZ = 1 + rand() % 21 - 10;  // 23 grids in z-axis
    dst[0] = numX * 1000;
    dst[1] = numZ * 1000;
    int x_distance = dst[0] - _pos[0];
    int y_distance = dst[1] - _pos[1];
    srand(time(0));
    // randomly decide where to break the route
    if (x_distance && y_distance)
    {
        cout << "setting break points" << endl;
        // when both x_distance and y_distance is greater than 0
        int random_num = rand() % abs(x_distance);
        _route = (int**)malloc(sizeof(int*) * 4);
        for (int i = 0; i < 4; i++)
            _route[i] = (int*)malloc(sizeof(int) * 2);
        //
        for (int i = 0; i < 2; i++)
            _route[0][i] = _pos[i];
        _task_size = 4;
        if (x_distance > 0)
            _route[1][0] = _pos[0] + random_num;
        else
            _route[1][0] = _pos[0] - random_num;
        _route[1][1] = _pos[1];
        _route[2][0] = _route[1][0];
        _route[2][1] = _route[1][1] + y_distance;
        _route[3][0] = dst[0];
        _route[3][1] = dst[1];
    }
    else
    {
        cout << "routing" << endl;
        _task_size = 2;
        _route = (int**)malloc(sizeof(int*) * _task_size);
        for (int i = 0; i < 2; i++)
            _route[i] = (int*)malloc(sizeof(int) * 2);
        for (int i = 0; i < 2; i++)
        {
            _route[0][i] = _pos[i];
            _route[1][i] = dst[i];
        }
    }
}


void Slave::update_state(int state)
{
    _STATE = state;
}


void Slave::update_abs(int* pos)
{
    if (_STATE == 1)
    {
        if (_direction == 'N' || _direction == 'S')
            _dist = abs(_route[_ptr][1] - pos[1]);
        else
            _dist = abs(_route[_ptr][0] - pos[0]);
        _dist = (_dist < 0) ? 0 : _dist;
    }
    if (_STATE != 2)
    {
        // only update the orientation when the state is not at 2
        _pos[2] = pos[2];
    }
    for (int i = 0; i < 2; i++)
        _pos[i] = pos[i];
}

int** Slave::get_route()
{
    return _route;
}

int Slave::forward(int _dist, char dir, int error)
{
    if (!is_moving)
    {
        prev_x = _pos[0];
        prev_z = _pos[1];
        is_moving = true;
        // cout << "Start forward" << endl;
    }
    int diff;

    if (dir == 'x')
    {
        diff = abs(prev_x - gp->getValues()[0] * scale);
    }
    else {
        diff = abs(prev_z - gp->getValues()[2] * scale);
    }

    if (diff < _dist)
    {
        calc_error();
        float gap = _pid.pid_compute(_error);
        motors[0]->setVelocity(maxSpeed - gap);
        motors[1]->setVelocity(maxSpeed + gap);
        return 0;
    }
    else {
        is_moving = false;
        motors[0]->setVelocity(0.0);
        motors[1]->setVelocity(0.0);
        // cout << "fianl Forward diff: " << diff << endl;
        // cout << "final Forward dist: " << _dist << endl;
        // cout << "End forward" << endl;
        return 1;
    }
}
int Slave::turn(int deg, char dir)
{
    float l_speed;
    float r_speed;
    const double* north = cp->getValues();
    float rad = atan2(north[0], north[2]);
    float bearing = (rad + 2 * 1.5708) / M_PI * 180.0;

    if (!is_moving) {
        prev_ori = bearing;
        is_moving = true;
    }

    float diff = abs(prev_ori - bearing);
    float gap = 0.0007 * (deg - diff);
    // cout << "turn gap : " << gap << endl;
    // cout << "turn diff: " << diff << endl;
    // cout << "turn deg : " << deg << endl;

    if (diff < deg) {
        if (dir == 'L') {
            l_speed = -0.5 - gap;
            r_speed = 0.5 + gap;
        }
        else {
            l_speed = 0.5 + gap;
            r_speed = -0.5 - gap;
        }
        motors[0]->setVelocity(l_speed);
        motors[1]->setVelocity(r_speed);
        return 0;
    }
    else {
        motors[0]->setVelocity(0.0);
        motors[1]->setVelocity(0.0);
        is_moving = false;
        // cout << "turn gap: " << (float) deg - diff << endl;
        // cout << "###################################" << endl;
        return 1;
    }
}


void Slave::main_executor()
{
    while (step(timeStep) != -1)
    {
        if (!start)
        {
            setup();
            start = true;
            cout << "Setup completed!" << endl;
        }

        int proceed = 0;
        switch (_STATE)
        {
        case 0:
            check_task();

            if (_STATE != 4)
            {
                action_decoder();
            }
            break;
        case 1:
            proceed = forward(_dist, forward_dir, _error);
            //      Serial.println("Moving forward");
            break;
        case 2:
            proceed = turn(_turn, 'L');
            //      Serial.println("turning left");
            break;
        case 3:
            proceed = turn(_turn, 'R');
            //      Serial.println("turning right");
            break;
        case 4:
            //      Serial.println("arrived");
            if (!arrived) {
                cout << getName() << " arrived at (" << dst[0];
                cout << "," << dst[1] << ")" << endl;
                arrived = true;
                cout << "================================" << endl;
            }
            break;
        }
        if (proceed)
            _STATE = 0;
        // update_est();
    }
}

void Slave::action_decoder()
{

    int* pos = get_pos();
    int x = _route[_ptr][0] - pos[0];
    int y = _route[_ptr][1] - pos[1];
    int ORI = _pos[2];
    free(pos);
    _turn = 90;
    // cout << "x diff: " << x << endl;
    // cout << "y diff: " << y << endl;
    if ((ORI <= 5) | (ORI >= 355))
    {
        if (y >= 200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (y <= -1)
        {
            _STATE = 1;
            _dist = abs(y);
            forward_dir = 'z';
            _direction = 'N';
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
    else if (ORI >= 175 && ORI <= 185)
    {
        if (y >= 1)
        {
            _STATE = 1;
            _dist = abs(y);
            forward_dir = 'z';
            _direction = 'S';
        }
        else if (y <= -200)
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
            _dist = abs(x);
            forward_dir = 'x';
            _direction = 'E';
        }
        else if (x <= -200)
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
    else if (ORI >= 265 && ORI <= 275)
    {
        if (x >= 200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (x <= -1)
        {
            _STATE = 1;
            _dist = abs(x);
            forward_dir = 'x';
            _direction = 'W';
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
            _STATE = (remain > 180) ? 2 : 3;
            _turn = 90 - abs(remain);
        }
        else
        {
            _STATE = (remain < 180) ? 2 : 3;
            _turn = abs(remain);
        }

    }

}

void Slave::check_task()
{
    int* pos = get_pos();

    if (abs(pos[0] - _route[_ptr][0]) <= 300 && abs(pos[1] - _route[_ptr][1]) <= 300)
    {
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
            cout << "arrived at a checkpoint (" << _route[_ptr][0] << "," << _route[_ptr][1] << ")" << endl;
            cout << "=============================" << endl;
            _ptr += 1;
        }
    }
    else {
        // cout << "=========================" << endl;
        // cout << "not even close" << endl;
        // cout << "p_x  : " << p_x << endl;
        // cout << "p_y  : " << p_y << endl;
        // cout << "x    : " << pos[0] << endl;
        // cout << "y    : " << pos[1] << endl;
        // cout << "STATE: " << _STATE << endl;
        // cout << "=========================" << endl;
    }
    free(pos);

}


void Slave::setup() {
    get_pos();
    cout << "Position intialized" << endl;
    auto_route();
    int** route = get_route();
    for (int i = 0; i < get_size(); i++)
    {
        cout << "ROUTE: " << route[i][0] << " " << route[i][1] << endl;
    }
    calc_error();
    cout << "=====================" << endl;
}




int main() {
    Slave* controller = new Slave();
    cout << "robot started" << endl;
    controller->main_executor();
    delete controller;
    return 0;
}
