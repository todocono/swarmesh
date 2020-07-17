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

float POS;
int ORI;
int TURN;
int new_ORI;
int STATE = 0; //0-STOP; 1-MOVE_FORWARD; 2-LEFT; 3-RIGHT;
const double pi = 3.1415926;

class Slave : public Robot {
public:
    Slave();
    void run();

private:

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



void Slave::run() {
    // main loop
    bool isMoving = false;
    
    float prev_x;
    float prev_y;

    while (step(timeStep) != -1)
    {
        if (!isMoving) {
            prev_x = gp->getValues()[0];
            prev_y = gp->getValues()[2];
        }
        
        if (abs(gp->getValues()[2]-prev_y) < 0.1) {
            motors[0]->setVelocity(maxSpeed);
            motors[1]->setVelocity(maxSpeed);
            isMoving = true;
        } else {
            motors[0]->setVelocity(0);
            motors[1]->setVelocity(0);
            cout << "X: " << gp->getValues()[0] << endl;
            cout << "Z: " << gp->getValues()[2] << endl;
        }
        
       
    }
}



int main() {
    Slave* controller = new Slave();
    controller->run();
    delete controller;
    return 0;
}
