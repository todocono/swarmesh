#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <time.h>

#define TIME_STEP 64
using namespace webots;
using namespace std;


int main(int argc, char **argv) {
  Robot *robot = new Robot();
  Motor *wheels[2];
  wheels[0] = robot->getMotor("left wheel motor");
  wheels[0]->setPosition(INFINITY);
  wheels[0]->setVelocity(0.0);
  wheels[1] = robot->getMotor("right wheel motor");
  wheels[1]->setPosition(INFINITY);
  wheels[1]->setVelocity(0.0);
  
  
  clock_t start = clock();
  double timeGap = (clock() - start) / CLOCKS_PER_SEC;
  bool isMoving = false;
  double speeds[2] = {0.0,0.0};
  
  cout << "Start" << endl;
  while (robot->step(TIME_STEP) != -1)
  {
       speeds[0] = 4.3;
       speeds[1] = 4.3;
       wheels[0]->setVelocity(speeds[0]);
       wheels[1]->setVelocity(speeds[1]);
       if (!isMoving)
       {
           start = clock();
           isMoving = true;
       }
       timeGap = (clock() - start) / CLOCKS_PER_SEC;
       if (timeGap >= 1)
       {
           isMoving = false;
           speeds[0] = 0.0;
           speeds[1] = 0.0;
           wheels[0]->setVelocity(speeds[0]);
           wheels[1]->setVelocity(speeds[1]);
           break;
       }  
  }

  delete robot;
  return 0;  // EXIT_SUCCESS
}