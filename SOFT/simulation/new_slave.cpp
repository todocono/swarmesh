#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Field.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Compass.hpp>
// #include <webots/Emitter.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#incldue <cmath>

using namespace std;
using namespace webots;

static const double maxSpeed = 10.0;
int *POS;
int *ORI;
const string *ID;
const double = 3.14159;

class Slave : public Robot {
public:
    Slave();
    void set_class();
    void load_dst(DynamicJsonDocument& jTask, int* POS);
    void select_dst(int* POS);
    void proceed_dst(DynamicJsonDocument& jDst, int* POS);
    int* get_dst();
    int arrive_dst(int* POS);
    void run();

private:
    enum Mode { STOP, TURN, MOVE_FORWARD, AVOID_OBSTACLES };

    static double boundSpeed(double speed);

    int timeStep;
    long timeTime;

    float* _init_pos;
    int* _dst;
    int** _dst_lst;
    int _lst_size;
    void _del_dst_lst(int idx);

    Mode mode;
    Receiver* receiver;
    Camera* camera;
    DistanceSensor* distanceSensors[2];
    Motor* motors[2];
    PositionSensor* ps[2];
    GPS* gp;
    InertialUnit* iu;
    Field* translationField;
    Compass* cp;
    // Emitter *emitter;
};

Slave::Slave() {
    timeStep = 32;
    timeTime = 32;
    mode = STOP;

    gp = getGPS("global");
    gp->enable(timeStep);

    iu = getInertialUnit("imu");
    iu->enable(timeStep);

    cp = getCompass("cp");
    cp->enable(timeStep);

    camera = getCamera("camera");
    camera->enable(4 * timeStep);
    receiver = getReceiver("receiver");
    receiver->enable(timeStep);
    motors[0] = getMotor("left wheel motor");
    motors[1] = getMotor("right wheel motor");
    motors[0]->setPosition(std::numeric_limits<double>::infinity());
    motors[1]->setPosition(std::numeric_limits<double>::infinity());
    motors[0]->setVelocity(0.0);
    motors[1]->setVelocity(0.0);

    ps[0] = getPositionSensor("left wheel sensor");
    ps[1] = getPositionSensor("right wheel sensor");
    ps[0]->enable(timeStep);
    ps[1]->enable(timeStep);

    string distanceSensorNames("ds0");
    for (int i = 0; i < 2; i++) {
        distanceSensors[i] = getDistanceSensor(distanceSensorNames);
        distanceSensors[i]->enable(timeStep);
        distanceSensorNames[2]++;  // for getting "ds1","ds2",...
    }
}

double Slave::boundSpeed(double speed) {
    return std::min(maxSpeed, std::max(-maxSpeed, speed));
}


void Slave::set_class()
{
    _dst = (int*)malloc(sizeof(int) * 2);
    _init_pos = (int*)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
    {
        _dst[i] = -1;
    }
}


void Destinations::_del_dst_lst(int idx)
{
    // swap the value at index with the last element
    _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
    _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
    // modify _dst_lst and get rid of the smallest element
    _lst_size--;
    cout << "Reduced List Size: " << _lst_size << endl;
    int** new_ptr = (int**)malloc(sizeof(int*) * (_lst_size));
    for (int i = 0; i < _lst_size; i++)
    {
        new_ptr[i] = (int*)malloc(sizeof(int) * 2);
        for (int j = 0; j < 2; j++)
        {
            new_ptr[i][j] = _dst_lst[i][j];
        }
        free(_dst_lst[i]);
    }
    free(_dst_lst);
    _dst_lst = new_ptr;
}


void Destinations::load_dst(DynamicJsonDocument& jTask, int *POS) // revise this line
{
    for (int i = 0; i < 2; i++) _init_pos[i] = POS[i];
    _lst_size = jTask["Num"]; // revise this line
    cout << "Task size: " << _lst_size << endl;
    _dst_lst = (int**)malloc(sizeof(int*) * _lst_size);
    for (int i = 0; i < _lst_size; i++)
    {
        _dst_lst[i] = (int*)malloc(sizeof(int) * 2);
        for (int j = 0; j < 2; j++)
        {
            _dst_lst[i][j] = jTask["Task"][i][j]; // revise this line
        }
    }
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
    for (int i = 0; i < 2; i++) _dst[i] = _dst_lst[idx][i];
    _del_dst_lst(idx);
}


void Destinations::proceed_dst(DynamicJsonDocument& jDst, int *POS) // revise this line
{
    int* dst = (int *)malloc(sizeof(int) * 2);
    dst[0] = jDst["Pos"][0];  // revise this line
    dst[1] = jDst["Pos"][1];  // revise this line
    cout << _lst_size << endl;
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
                cout << "matched" << endl;
            }
            // filter out destination of robots going back to their origin
            if (idx >= 0) _del_dst_lst(idx);
        }
    }
    else
    {
        // run out of available Destinations
        // go back to original point
        //    only doing so when the last destination conflicts the destination taken by others
        if (dst[0] == _dst[0] && dst[1] == _dst[1]) {
            for (int i = 0; i < 2; i++) _dst[i] = _init_pos[i];
        }
    }
}


int* Destinations::get_dst() {
    int* new_ptr = (int*)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
    {
        new_ptr[i] = _dst[i];
    }
    return new_ptr;
}

int Destinations::arrive_dst(int* POS)
{
    if (POS[0] == _dst[0] && POS[1] == _dst[1])
    {
        return 1;
    }
    return 0;
}


void Slave::run() {
    // main loop
    POS = (int*)malloc(sizeof(int) * 2);
    ID = getName();
    set_class();
    while (step(timeStep) != -1) {
        // Update position
        POS[0] = round(gp->getValues()[0] / 0.1); // X coordinate
        POS[1] = round(gp->getValues()[2] / 0.1); // Z coordinate

        // Update ori -> range(-180,180)
        const double* north = cp->getValues();
        double rad = atan2(north[0], north[2]);
        double bearing = (rad - 1.5708) / M_PI * 180.0;
        if (bearing < 0.0)
        { 
            bearing = bearing + 360.0;
        }
        else if (bearing == 360.0)
        {
            bearing = bearing - 360.0;
        }
        ORI = bearing;

        if (receiver->getQueueLength() > 0)
        { 
            string message((const char*)receiver->getData());
            receiver->nextPacket();
            cout << getName() << " Points are " << message << "!" << endl;

            string strPurpose;
            strPurpose = message[0]
            const int Purpose = ctoi(strPurpose);

            switch (Purpose)
            {
            case 1:
                {
                cout << "Tasks received" << endl;
                mode = MOVE_FORWARD;
                load_dst(jInfo, POS);    // revise this line
                break;
                }
            case 2:
                {
                // other robots have arrived at their destinations
                // need to check whether the destinations are the same as its own destination
                // move only when it self is not at the destination
                if (!arrive_dst(POS))
                    {
                    proceed_dst(jInfo, POS);  // revise this line
                    }
                }
            }
        else    // Normal move 
        {




        }
            // if (message.compare("avoid obstacles") == 0)
              // mode = AVOID_OBSTACLES;
            // else if (message.compare("move forward") == 0)
              // mode = MOVE_FORWARD;
            // else if (message.compare("stop") == 0)
              // mode = STOP;
            // else if (message.compare("turn") == 0)
              // mode = TURN;
            // else if (message.compare("order") == 0 and getName().compare("robot1") == 0)
              // mode = ORDER;
        }



        //double delta = distanceSensors[0]->getValue() - distanceSensors[1]->getValue();
        double speeds[2] = { 0.0, 0.0 };

        // send actuators commands according to the mode
        switch (mode) {
        //case AVOID_OBSTACLES:
            //speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
            //speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
            //break;
        case MOVE_FORWARD:
            speeds[0] = maxSpeed;
            speeds[1] = maxSpeed;
            break;
        case TURN:
            speeds[0] = maxSpeed / 2.0;
            speeds[1] = -maxSpeed / 2.0;
            break;
        default:
            break;
        }
        motors[0]->setVelocity(speeds[0]);
        motors[1]->setVelocity(speeds[1]);
    }
}

int main() {
    Slave* controller = new Slave();
    controller->run();
    delete controller;
    return 0;
}
