#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>
#include <cstdlib>
#include <ctime>


using namespace std;
using namespace webots;

class Driver : public Supervisor {
public:
  Driver();
  void run();

private:
  int timeStep;
  Emitter *emitter;
  Field *translationField;
  double x;
  double z;
  double translation[3];
}; 

Driver::Driver() {
  timeStep = 128;
  x = -0.25f;
  z = -0.25f;

  translation[0] = x;
  translation[2] = z;
  emitter = getEmitter("emitter");
  // Node *robot = getFromDef("ROBOT1");
  // cout << "Haha " << robot->getId() << endl;
  // if (!robot) {
    // robot might be NULL if the controller is about to quit
    // exit(1);
  // }
  // translationField = robot->getField("rotation");
}


void Driver::run() 
{
  // const double *RotationValues = translationField->getSFRotation();
  // cout << RotationValues[3] << endl;
  string message("1 3 ");
  int numsX[3]; // 3(num of robots) random x-coordinates for dst
  int numsZ[3]; // 3(num of robots) random z-coordinates for dst
  srand(time(0));
  for (int i = 0; i < 3; i++) 
  {
      numsX[i] = 1 + rand()%26 - 14;  // 30 grids in x-axis
      // cout << numsX[i] << " in x" << endl;
      numsZ[i] = 1 + rand()%19 - 10;  // 23 grids in z-axis
      // cout << numsZ[i] << " in z" << endl; 
  }
  for (int i = 0; i < 3; i++)
  {
      string temp1=to_string(numsX[i]);
      string temp2=to_string(numsZ[i]);
      message.append(temp1);
      message.append(",");
      message.append(temp2);
      message.append(" ");
    
  }

   // send actuators commands; send a new message through the emitter device
   cout << "Random points are: " << message.c_str() << endl;
   emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
  
}


int main() {
  Driver *controller = new Driver();
  controller->run();
  delete controller;
  return 0;
}
