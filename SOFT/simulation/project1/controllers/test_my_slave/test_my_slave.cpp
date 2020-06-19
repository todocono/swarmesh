#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Compass.hpp>
// #include <webots/Emitter.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <cmath>
#include <time.h>
#include <cstring>

using namespace std;
using namespace webots;

static const double maxSpeed = 0.1;

class Slave : public Robot {
public:
  Slave();
  void run();

private:
  enum Mode { STOP, MOVE_FORWARD, AVOID_OBSTACLES, TURN };

  static double boundSpeed(double speed);

  int timeStep;
  long timeTime;
  Mode mode;
  Receiver *receiver;
  Camera *camera;
  DistanceSensor *distanceSensors[2];
  Motor *motors[2];
  PositionSensor *ps[2];
  Compass *cp;
  // Emitter *emitter;
};

Slave::Slave() {
  timeStep = 32;
  timeTime = 32;
  mode = STOP;
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

  cp = getCompass("cp");
  cp->enable(timeStep);
  
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

void Slave::run() 
{
  // main loop
  while (step(timeStep) != -1) 
  {
    // Read sensors, particularly the order of the supervisor
    // if (receiver->getQueueLength() > 0) {
      // string message((const char *)receiver->getData());
      // cout << message[0] << endl;
      // receiver->nextPacket();

      // cout << "I should " << message << "!" << endl;
      
      
      // cout << motors[0]->getTargetPosition() << endl;
      // cout << motors[1]->getTargetPosition() << endl;
      // cout << getName() << " wheel L " << ps[0]->getValue() << endl;
      // cout << getName() << " wheel R " << ps[1]->getValue() << endl;
      // cout << getName() << " channel: " << receiver->getChannel() << endl;
      
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
    // }
    mode = TURN;
    const double *north = cp->getValues();
    double rad = atan2(north[0], north[2]);
    double bearing = (rad + 1.5708*2) / 3.1415926 * 180.0;
    double delta = distanceSensors[0]->getValue() - distanceSensors[1]->getValue();
    double speeds[2] = {0.0, 0.0};

    // send actuators commands according to the mode
    switch (mode) 
    {
      case AVOID_OBSTACLES:
        speeds[0] = boundSpeed(maxSpeed / 2.0 + 0.1 * delta);
        speeds[1] = boundSpeed(maxSpeed / 2.0 - 0.1 * delta);
        break;
      case MOVE_FORWARD:
        speeds[0] = maxSpeed;
        speeds[1] = maxSpeed;
        break;
      case TURN:
        cout << bearing << endl;
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
  Slave *controller = new Slave();
  controller->run();
  delete controller;
  return 0;
}
