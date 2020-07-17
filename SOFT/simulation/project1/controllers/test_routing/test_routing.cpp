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
static const double maxSpeed = 4.3;
const double pi = 3.1415926;
bool start = false;
bool arrived = false;
float dst[2];

class Slave : public Robot {
public:
    Slave();
    float* get_pos();
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
    void update_abs(float* pos);
    
    int forward(float _dist, char dir);
    int turn(int deg, char dir); 
    
    void setup();
    
private:
    int _ptr;
    int _turn;
    float _dist;
    int _error;
    int _STATE;
    int _width;
    int _task_size;
    int _off_course;
    int _dst[2];
    int _prev_tick[2];
    int** _route;
    float* _pos; // this contains the current coordinate as well as the rotation of the robot [x, y, u]
    
    bool is_moving = false;
    float prev_x;
    float prev_z;
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
    _pos = (float*)malloc(sizeof(float) * 3);
    
    
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


float* Slave::get_pos()
{
    const double* north = cp->getValues();
    float rad = atan2(north[0], north[2]);
    float bearing = (rad + 2 * 1.5708) / M_PI * 180.0;
    _pos[0] = gp->getValues()[0];
    _pos[1] = gp->getValues()[2];
    _pos[2] = bearing;
    float* ptr = (float*)malloc(sizeof(float) * 3);
    ptr[0] = gp->getValues()[0];
    ptr[1] = gp->getValues()[2];
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

void Slave::auto_route()
{
    // A* routing algorithm goes here
    dst[0] = 0.4;
    dst[1] = 0.5;
    int x_distance = (dst[0] / 0.1) - round(_pos[0] * 10);
    int y_distance = (dst[1] / 0.1) - round(_pos[1] * 10);
    srand(time(0));
    // randomly decide where to break the route
    if (x_distance && y_distance)
    {
        cout << "setting break points" << endl;
        // when both x_distance and y_distance is greater than 0
        int random_num = rand() % abs(x_distance);
        _route = (int**)malloc(sizeof(int*) * 4);
        for (int i = 0; i <  4; i++)
            _route[i] = (int*)malloc(sizeof(int) * 2);
        //
        for (int i = 0; i < 2; i++)
            _route[0][i] = round(_pos[i] / 0.1);
        _task_size = 4;
        if (x_distance > 0)
            _route[1][0] = round(_pos[0] / 0.1) + random_num;
        else
            _route[1][0] = round(_pos[0] / 0.1) - random_num;
        _route[1][1] = round(_pos[1] / 0.1);
        _route[2][0] = _route[1][0];
        _route[2][1] = _route[1][1] + y_distance * 1;
        _route[3][0] = round(dst[0] / 0.1);
        _route[3][1] = round(dst[1] / 0.1);
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
            _route[0][i] = round(_pos[i] / 0.1);
            _route[1][i] = round(dst[i] / 0.1);
        }
    }
}

void Slave::update_abs(float* pos)
{
    for (int i = 0; i < 3; i++)
        _pos[i] = pos[i];
}

int** Slave::get_route()
{
    return _route;
}

int Slave::forward(float _dist, char dir)
{
    if (!is_moving) 
    {
        prev_x = _pos[0];
        prev_z = _pos[1];
        is_moving = true;
        cout << "Start forward" << endl;
    }
    float diff;
    
    if (dir == 'x')
    {
        diff = abs(prev_x - gp->getValues()[0]);
    } else {
        diff = abs(prev_z - gp->getValues()[2]);
    }
    
    if (diff < _dist)
    {
        motors[0]->setVelocity(maxSpeed);
        motors[1]->setVelocity(maxSpeed);
        return 0;
    } else {
        is_moving = false;
        motors[0]->setVelocity(0.0);
        motors[1]->setVelocity(0.0);
        cout << "fianl Forward diff: " << diff << endl;
        cout << "final Forward dist: " << _dist << endl;
        cout << "End forward" << endl;
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

    if (diff < deg) {
        if (dir == 'L') {
            l_speed = -0.5;
            r_speed = 0.5;
        } else {
            l_speed = 0.5;
            r_speed = -0.5;
        }
        motors[0]->setVelocity(l_speed);
        motors[1]->setVelocity(r_speed);
        return 0;
    } else {
        motors[0]->setVelocity(0.0);
        motors[1]->setVelocity(0.0);
        is_moving = false;
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
            proceed = forward(_dist, forward_dir);
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
                cout << "================================" << endl;
                cout << getName() << " arrived at (" << dst[0];
                cout << "," << dst[1] << ")" << endl;
                arrived = true;
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

    float* pos = get_pos();
    float route_x = _route[_ptr][0];
    float route_y = _route[_ptr][1];
    // cout << "action decoder +++++++++++++++++++" << endl;
    // cout << "route_x: " << route_x << endl;
    // cout << "route_y: " << route_y << endl;
    float precise_x = (route_x / 10 - pos[0]);
    float precise_y = (route_y / 10 - pos[1]);
    int x = round(precise_x * 10);
    int y = round(precise_y * 10);
    // cout << "precise_x: " << precise_x << endl;
    // cout << "precise_y: " << precise_y << endl;
    // cout << "int_x: " << x << endl;
    // cout << "int_y: " << y << endl;
    // cout << "ori: " << pos[2] << endl;
    // cout << "+++++++++++++++++++++++++++++++++" << endl;
    
    free(pos);
    int ORI = _pos[2];
    _turn = 90;
    if ((ORI <= 5) | (ORI >= 355))
    {
        if (y >= 1)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (y <= -1)
        {
            _STATE = 1;
            _dist = abs(precise_y);
            forward_dir = 'z';
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
            _dist = abs(precise_y);
            forward_dir = 'z';
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
            _dist = abs(precise_x);
            forward_dir = 'x';
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
    else if (ORI >= 265 && ORI <= 275)
    {
        if (x >= 1)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (x <= -1)
        {
            _STATE = 1;
            _dist = abs(precise_x);
            forward_dir = 'x';
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
    float* pos = get_pos();
    float p_x = _route[_ptr][0];
    float p_y = _route[_ptr][1];
    
    if (abs(pos[0] - p_x / 10) <= 0.05 && abs(pos[1] - p_y / 10) <= 0.05)
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
            cout << "arrived at a checkpoint" << endl;
            _ptr += 1;
        }
    } else {
        cout << "=========================" << endl;
        cout << "not even close" << endl;
        cout << "p_x  : " << p_x << endl;
        cout << "p_y  : " << p_y << endl;
        cout << "x    : " << pos[0] << endl;
        cout << "y    : " << pos[1] << endl;
        cout << "STATE: " << _STATE << endl;
        cout << "=========================" << endl;
    }
    free(pos);
    
}


void Slave::setup() {
    float pos[3];
    const double* north = cp->getValues();
    float rad = atan2(north[0], north[2]);
    float bearing = (rad + 2 * 1.5708) / M_PI * 180.0;
    pos[0] = gp->getValues()[0]; // X coordinate
    pos[1] = gp->getValues()[2]; // Z coordinate
    pos[2] = bearing;
    update_abs(pos);
    cout << "Position intialized" << endl;
    auto_route();
    int** route = get_route();
    for (int i = 0; i < get_size(); i++)
    {
        cout << "ROUTE: " << route[i][0] << " " << route[i][1] << endl;
    }

}




int main() {
    Slave* controller = new Slave();
    cout <<  "robot started" << endl;
    controller->main_executor();
    delete controller;
    return 0;
}
