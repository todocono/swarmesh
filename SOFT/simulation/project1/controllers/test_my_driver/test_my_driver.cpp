#include <webots/Emitter.hpp>
#include <webots/Field.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <string>


using namespace std;
using namespace webots;

class Driver : public Supervisor {
public:
  Driver();
  void run();

private:
  static void displayHelp();
  int timeStep;
  Emitter *emitter;
  Field *translationField;
  Keyboard *keyboard;
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
  Node *robot = getFromDef("ROBOT1");
  if (!robot){
    // robot might be NULL if the controller is about to quit
    exit(1);
  }
  translationField = robot->getField("translation");
  keyboard = getKeyboard();
  keyboard->enable(timeStep);
}

void Driver::run() {
  string previous_message("");
  string message("");

  displayHelp();
  float orderX = -0.25;
  float orderZ = -0.25;
  // cout << emitter->getChannel() << endl;
  // main loop
  while (step(timeStep) != -1) {
    // Read sensors; update message according to the pressed keyboard key
    int k = keyboard->getKey();
    switch (k) {
      case 'A':
        message.assign("avoid obstacles");
        break;
      case 'F':
        message.assign("move forward");
        break;
      case 'S':
        message.assign("stop");
        break;
      case 'T':
        message.assign("turn");
        break;
      case 'I':
        displayHelp();
        break;
      case 'G': {
        const double *translationValues = translationField->getSFVec3f();
        cout << "ROBOT1 is located at (" << translationValues[0] << "," << translationValues[2] << ")" << endl;
        break;
      }
      case 'R':
        cout << "Teleport ROBOT1 at (" << x << "," << z << ")" << endl;
        translationField->setSFVec3f(translation);
        break;
      case 'O':
        cout << "Robot1 go to (" << orderX << "," << orderZ << ")" << endl;
        message.assign("order");
        break;
      default:
        message.clear();
    }

    // send actuators commands; send a new message through the emitter device
    if (!message.empty() && message.compare(previous_message)) {
      previous_message.assign(message);
      cout << "Please, " << message.c_str() << endl;
      emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
    }
  }
}

void Driver::displayHelp() {
  string s("Commands:\n"
           " I for displaying the commands\n"
           " A for avoid obstacles\n"
           " F for move forward\n"
           " S for stop\n"
           " T for turn\n"
           " R for positioning ROBOT1 at (0.1,0.3)\n"
           " G for knowing the (x,z) position of ROBOT1");
  cout << s << endl;
}

int main() {
  Driver *controller = new Driver();
  controller->run();
  delete controller;
  return 0;
}