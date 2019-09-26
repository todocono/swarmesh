//************************************************************
// this is a simple example that uses the painlessMesh library
//
// 1. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 2. prints anything it receives to Serial.print
//
//
//************************************************************



#include <FastLED.h>
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555


#define DATA_PIN    25
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];// Define the array of leds

unsigned long previousMillis1 = 0;        // will store last time LED was updated
unsigned long previousMillis2 = 0;        // will store last time LED was updated
const long debounce = 500;           // interval at which to blink (milliseconds)
unsigned int led = 0;

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

// User stub
//void sendMessage() ; // Prototype so PlatformIO doesn't complain

//Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

void sendMessage1() {
  Serial.println("hello broadcast");
  String msg = "Hello from node ";
  msg += mesh.getNodeId();
  mesh.sendBroadcast( msg );
  // taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void sendMessage2() {
  Serial.println("bye broadcast");
  String msg = "bye from node ";
  msg += mesh.getNodeId();
  mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  leds[led] = CRGB::White; //add an LED
  if (led < 64)    led++;
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}


void gotTouch1() {
  // touch1detected = true;
  if ((millis() - previousMillis1) > debounce) {
    sendMessage1();
    previousMillis1 = millis();
  }
}


void gotTouch2() {
  // touch2detected = true;
  if ((millis() - previousMillis2) > debounce) {
    sendMessage2();
    previousMillis2 = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("starting...");
  pinMode( 12, OUTPUT);
  pinMode( 13, OUTPUT);
  pinMode( 14, OUTPUT);
  pinMode( 27, OUTPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // userScheduler.addTask( taskSendMessage );
  // taskSendMessage.enable();
  touchAttachInterrupt(T2, gotTouch1, 40);
  touchAttachInterrupt(T3, gotTouch2, 40);

}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  FastLED.show();
}
