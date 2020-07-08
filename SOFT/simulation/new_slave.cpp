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

using namespace std;
using namespace webots;

static const double maxSpeed = 4.3;

int* POS;
int ORI;
int TURN;
int new_ORI;
int STATE = 0; //0-STOP; 1-MOVE_FORWARD; 2-LEFT; 3-RIGHT;
const double pi = 3.1415926;

class Slave : public Robot {
public:
    Slave();
    void set_class();
    void actionDecoder(int ORI, int* POS, int* DST);
    void load_dst(string jTask, int* POS);
    void select_dst(int* POS);
    void proceed_dst(string jDst, int* POS);
    int* get_dst();
    int arrive_dst(int* POS);
    void run();
    int sendTimes = 0;

private:

    static double boundSpeed(double speed);

    int timeStep;

    int* _init_pos;
    int* _dst;
    int** _dst_lst;
    int _lst_size;
    void _del_dst_lst(int idx);

    Receiver* receiver;
    // DistanceSensor* distanceSensors[2];
    Motor* motors[2];
    GPS* gp;
    Compass* cp;
    Emitter* emitter;
};

Slave::Slave() {
    timeStep = 32;

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

void Slave::set_class()
{
    _dst = (int*)malloc(sizeof(int) * 2);
    _init_pos = (int*)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
    {
        _dst[i] = -1;
    }
}


void Slave::_del_dst_lst(int idx)
{
    // swap the value at index with the last element
    _dst_lst[idx][0] = _dst_lst[_lst_size - 1][0];
    _dst_lst[idx][1] = _dst_lst[_lst_size - 1][1];
    // modify _dst_lst and get rid of the smallest element
    _lst_size--;

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


void Slave::load_dst(string jTask, int* POS)
{
    for (int i = 0; i < 2; i++) _init_pos[i] = POS[i];
    string size_first(1, jTask[2]);
    string size_second(1, jTask[3]);
    _lst_size = stoi(size_first + size_second);

    _dst_lst = (int**)malloc(sizeof(int*) * _lst_size);
    unsigned int i = 5;
    int idx = 0;
    int x;
    int z;
    string temp("");

    for (int i = 0; i < _lst_size; i++)
    {
        _dst_lst[i] = (int*)malloc(sizeof(int) * 2);
    }

    while (i < jTask.length())
    {
        string thisChar(1, jTask[i]);
        if (thisChar.compare(" ") == 0)
        {
            z = stoi(temp);
            _dst_lst[idx][0] = x;
            _dst_lst[idx][1] = z;
            idx = idx + 1;
            temp = "";
        }
        else if (thisChar.compare(",") == 0)
        {
            x = stoi(temp);
            temp = "";
        }
        else
        {
            temp.append(thisChar);
        }
        i = i + 1;
    }
    select_dst(POS);
}


void Slave::select_dst(int* POS)
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
    cout << getName() << " go to: (";
    for (int i = 0; i < 2; i++)
    {
        _dst[i] = _dst_lst[idx][i];
        cout << _dst[i] << ",";
    }
    cout << ")" << endl;
    _del_dst_lst(idx);
}


void Slave::proceed_dst(string jDst, int* POS)
{
    int* dst = (int*)malloc(sizeof(int) * 2);
    // 3_X-coor,Z-coor_
    unsigned int i = 2;
    int x;
    int z;
    string temp("");
    while (i < jDst.length())
    {
        string thisChar(1, jDst[i]);
        if (thisChar.compare(" ") == 0)
        {
            z = stoi(temp);
            dst[0] = x;
            dst[1] = z;
            temp = "";
        }
        else if (thisChar.compare(",") == 0)
        {
            x = stoi(temp);
            temp = "";
        }
        else
        {
            temp.append(thisChar);
        }
        i = i + 1;
    }

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


int* Slave::get_dst() {
    int* new_ptr = (int*)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
    {
        new_ptr[i] = _dst[i];
    }
    return new_ptr;
}

int Slave::arrive_dst(int* POS)
{
    if (POS[0] == _dst[0] && POS[1] == _dst[1])
    {
        return 1;
    }
    return 0;
}


void Slave::run() {
    // main loop
    clock_t start = clock();
    bool isMoving = false;

    POS = (int*)malloc(sizeof(int) * 2);

    set_class();

    while (step(timeStep) != -1)
    {
        POS[0] = round(gp->getValues()[0] / 0.1); // X coordinate
        POS[1] = round(gp->getValues()[2] / 0.1); // Z coordinate

        const double* north = cp->getValues();
        double rad = atan2(north[0], north[2]);
        double bearing = (rad + 2 * 1.5708) / M_PI * 180.0;
        ORI = bearing;

        if (receiver->getQueueLength() > 0 && !isMoving)
        {
            string message((const char*)receiver->getData());
            receiver->nextPacket();

            string strPurpose;
            strPurpose = message[0];
            const int Purpose = stoi(strPurpose);


            switch (Purpose)
            {
            case 1:
            {
                cout << "Tasks received" << endl;
                load_dst(message, POS);
                break;
            }
            case 2:
            {
                // other robots have arrived at their destinations
                // need to check whether the destinations are the same as its own destination
                // move only when it self is not at the destination
                if (!arrive_dst(POS))
                {
                    proceed_dst(message, POS);
                    // cout << getName() << " get info " << message << endl;
                }
                break;
            }
            }
        }
        else    // Normal move 
        {
            if (!isMoving)
            {
                // Update position
                int* ptr = get_dst();
                int arrive = arrive_dst(POS);

                if (ptr[0] != -1 && !arrive)
                {

                    actionDecoder(ORI, POS, ptr);

                }
                else if (arrive)
                {
                    if (sendTimes < 1)
                    {
                        sendTimes += 1;
                        cout << getName() << " arrives at (" << POS[0] << "," << POS[1] << ")" << endl;
                        string sendMessage = "2 ";
                        string strX = to_string(POS[0]);
                        string strZ = to_string(POS[1]);
                        sendMessage.append(strX);
                        sendMessage.append(",");
                        sendMessage.append(strZ);
                        sendMessage.append(" ");
                        emitter->send(sendMessage.c_str(), (int)strlen(sendMessage.c_str()) + 1);
                    }
                }
            }

        }



        double speeds[2] = { 0.0, 0.0 };

        switch (STATE)
        {
        case 1:
            speeds[0] = maxSpeed;
            speeds[1] = maxSpeed;
            if (!isMoving)
            {
                start = clock();
                isMoving = true;
            }
            double timeGap;
            timeGap = (clock() - start) / CLOCKS_PER_SEC;
            if (timeGap >= 1)
            {
                STATE = 0;
                isMoving = false;
                speeds[0] = 0.0;
                speeds[1] = 0.0;
            }

            motors[0]->setVelocity(speeds[0]);
            motors[1]->setVelocity(speeds[1]);
            break;

        case 2:
            if (!isMoving)
            {
                isMoving = true;
            }

            if (abs(ORI - new_ORI) > 1.5)
            {
                speeds[0] = -0.5;
                speeds[1] = 0.5;
            }
            else
            {
                speeds[0] = 0.0;
                speeds[1] = 0.0;
                isMoving = false;
                STATE = 0;
            }


            motors[0]->setVelocity(speeds[0]);
            motors[1]->setVelocity(speeds[1]);
            break;

        case 3:
            if (!isMoving)
            {
                isMoving = true;
            }

            if (abs(ORI - new_ORI) > 1.5)
            {
                speeds[0] = 0.5;
                speeds[1] = -0.5;
            }
            else
            {
                speeds[0] = 0.0;
                speeds[1] = 0.0;
                isMoving = false;
                STATE = 0;

            }

            motors[0]->setVelocity(speeds[0]);
            motors[1]->setVelocity(speeds[1]);
            break;
        case 0:
            motors[0]->setVelocity(0.0);
            motors[1]->setVelocity(0.0);
            break;
        }
    }
}


void Slave::actionDecoder(int ORI, int* POS, int* DST)
{
    //  calculate the robots' absolute position
    int x = DST[0] - POS[0];
    int y = DST[1] - POS[1];
    if (abs(ORI) <= 3 || abs(ORI) >= 357)
    {
        if (y <= -1)
        {
            STATE = 1;
        }
        else if (y >= 1)
        {
            STATE = 3;
            new_ORI = 180;
        }
        else
        {
            if (x >= 1)
            {
                STATE = 3;
                new_ORI = 90;
            }
            else if (x <= -1)
            {
                STATE = 2;
                new_ORI = 270;
            }
        }
    }
    else if (ORI >= 87 && ORI <= 93)
    {
        if (x >= 1)
        {
            STATE = 1;
        }
        else if (x <= -1)
        {
            STATE = 3;
            new_ORI = 270;
        }
        else
        {
            if (y >= 1)
            {
                STATE = 3;
                new_ORI = 180;
            }
            else if (y <= -1)
            {
                STATE = 2;
                new_ORI = 0;
            }
        }
    }
    else if (ORI >= 177 && ORI <= 183)
    {
        if (y >= 1)
        {
            STATE = 1;
        }
        else if (y <= -1)
        {
            STATE = 3;
            new_ORI = 0;
        }
        else
        {
            if (x >= 1)
            {
                STATE = 2;
                new_ORI = 90;
            }
            else if (x <= -1)
            {
                STATE = 3;
                new_ORI = 270;
            }
        }
    }
    else if (ORI >= 267 && ORI <= 273)
    {
        if (x <= -1)
        {
            STATE = 1;
        }
        else if (x >= 1)
        {
            STATE = 3;
            new_ORI = 90;
        }

        else
        {
            if (y >= 1)
            {
                STATE = 2;
                new_ORI = 180;
            }
            else if (y <= -1)
            {
                STATE = 3;
                new_ORI = 0;
            }
        }
    }
    else
    {
        int remain = ORI % 90;

        if (ORI >= 0 && ORI <= 90)
        {
            if (remain <= 45)
            {
                STATE = 2;
                new_ORI = 0;
            }
            else
            {
                STATE = 3;
                new_ORI = 90;
            }
        }
        else if (ORI >= 90 && ORI <= 180)
        {
            if (remain <= 45)
            {
                STATE = 2;
                new_ORI = 90;
            }
            else
            {
                STATE = 3;
                new_ORI = 180;
            }
        }
        else if (ORI >= 180 && ORI <= 270)
        {
            if (remain <= 45)
            {
                STATE = 2;
                new_ORI = 180;
            }
            else
            {
                STATE = 3;
                new_ORI = 270;
            }
        }
        else
        {
            if (remain <= 45)
            {
                STATE = 2;
                new_ORI = 270;
            }
            else
            {
                STATE = 3;
                new_ORI = 360;
            }
        }
        //STATE = 3;
        //new_ORI = 0;
    }
}

int main() {
    Slave* controller = new Slave();
    controller->run();
    delete controller;
    return 0;
}
