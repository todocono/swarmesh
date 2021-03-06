#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (250)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


#include <FastLED.h>
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555


#define DATA_PIN    25
#define NUM_LEDS    64

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 312

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

CRGB leds[NUM_LEDS];// Define the array of leds

unsigned long previousMillis1 = 0;        // will store last time LED was updated
unsigned long previousMillis2 = 0;        // will store last time LED was updated
const long debounce = 500;           // interval at which to blink (milliseconds)
unsigned int led = 0;

int stateMotors = 0;
int stateLeds = 0;

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

String msg = "";
boolean bnoDetected = false;
int values[] = {0, 0, 0};

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated



// User stub
void sendMessage() ; // Prototype so PlatformIO doesn't complain

//Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

struct Tick {
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void IRAM_ATTR isr() {
  encoder1.numberTicks += 1;
  encoder1.tickOn = true;
}
void IRAM_ATTR isr2() {
  encoder2.numberTicks += 1;
  encoder2.tickOn = true;
}


void sendMessageForward() {
  // Serial.println("FORWARD");
  // String msg = "FORWARD";
  mesh.sendBroadcast( msg );
  // taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageBackward() {
  //  Serial.println("BACK");
  // String msg = "BACK";
  //  mesh.sendBroadcast( msg );
  // taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageRight() {
  //  Serial.println("RIGHT");
  // String msg = "RIGHT";
  //  mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageLeft() {
  // Serial.println("LEFT");
  // String msg = "LEFT";
  // mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageWhite() {
  // Serial.println("WHITE");
  // String msg = "WHITE";
  // mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageBlack() {
  //  Serial.println("BLACK");
  //  String msg = "BLACK";
  //  mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
void sendMessageRain() {
  //  Serial.println("RAIN");
  //  String msg = "RAIN";
  // mesh.sendBroadcast( msg );
  //taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}
// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  // Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());


  /*if (msg == "MOVEFORWARD") {
    stateMotors = 1;
    } else if (msg == "MOVERIGHT") {
    stateMotors = 2;
    } else if (msg == "MOVELEFT") {
    stateMotors = 3;
    } else if (msg == "MOVEBACKWARD") {
    stateMotors = 4;
    } else if (msg == "LEDBLACK") {
    stateLeds = 1;
    } else if (msg == "LEDWHITE") {
    stateLeds = 2;
    } else if (msg == "LEDRAIN") {
    stateLeds = 3;
    }
  */


  int counter  = 0;
  int lastIndex = 0;
  for (int i = 0; i < msg.length(); i++) {
    if (msg.substring(i, i + 1) == ",") {
      // Grab the piece from the last index up to the current position and store it
      values[counter] = msg.substring(lastIndex, i).toInt();
      // Update the last position and add 1, so it starts from the next character
      lastIndex = i + 1;
      // Increase the position in the array that we store into
      counter++;
    }
  }
  Serial.print( String("values received ") + values[0] + "," + values[1] + "," + values[2]) + ",";

  float speedA = map(values[0], -90, 90, -255, 255); //speed
  float speedB = map(values[0], -90, 90, -255, 255); //speed
  float rate = map(values[1], -90, 90, -1, 1); //turning

//  if (rate > 0) {
//    speedA = speedA * (1 - rate);
//    speedB = speedB * rate;
//  } else if (rate < -0.2) {
//    motorA(speedA * -rate);
//    motorB(speedB * rate);
//  } else {
//    motorA(speedA);
//    motorB(speedB);
//  }
  motorA(speedA );
  motorB(speedB );

  leds[0].red = values[2]; //leds
  //  leds[led] = CRGB::White; //add an LED
  //  if (led < 64)    led++;
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
//
void setup() {
  Serial.begin(115200);
  Serial.println("starting...");
  pinMode( 12, OUTPUT);
  pinMode( 13, OUTPUT);
  pinMode( 14, OUTPUT);
  pinMode( 27, OUTPUT);

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(30);
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
  ledcAttachPin(PWMB, 2);
  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);


  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;



  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bnoDetected = false;
  } else {
    bnoDetected = true;
    delay(1000);


    /* Display the current temperature */
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(false);

    Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  }
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  //checkEncoders(stateMotors);
  //checkLeds(stateLeds);

  FastLED.show();

  if (  bnoDetected) {
    bnoSend();
  }

  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    int incomingByte = Serial.read();
    if (incomingByte == 'w') {
      msg = "MOVEFORWARD";
    } else  if (incomingByte == 's') {
      msg = "MOVEBACKWARD";
    } else    if (incomingByte == 'd') {
      msg = "MOVERIGHT";
    } else    if (incomingByte == 'a') {
      msg = "MOVELEFT";
    } else    if (incomingByte == 'j') {
      msg = "LEDWHITE";
    } else    if (incomingByte == 'k') {
      msg = "LEDBLACK";
    } else    if (incomingByte == 'l') {
      msg = "LEDRAIN";
    } else {
      msg = "";
    }
    Serial.println(msg);
    mesh.sendBroadcast( msg );
  }
}

void checkEncoders(int y) { //
  //depending on which case, check the encoders and decide the motors movment
  //  if (y == 0) {
  //    return;
  //  } else if (y == 1) {
  //    // only deal with one state at a time
  //    //implement a state queue that stores all the future actions
  //    forward(1000);
  //    return;
  //  } else if (y == 2) {
  //    turnLeft(90);
  //    return;
  //  } else if (y == 3) {
  //    turnRight(90);
  //    return;
  //  } else if (y == 4) {
  //    backward(1000);
  //    return;
  //  }
}

void checkLeds(int x) { //
  //depending on which case, check the leds
  if (x == 0) {
    return;
  } else if (x == 1) {
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Black;
    }
    return;
  } else if (x == 2) {
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::White;
    }
    return;
  } else if (x == 3) {
    for ( int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Blue;
    }
    return;
  }
}
//
//void forward(int dist) {
//  if (encoder1.numberTicks < dist) {
//    if ( encoder1.numberTicks > encoder2.numberTicks) {
//      motorA (spd, 0);
//      motorB (0, 0);
//    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
//      motorA (0, 0);
//      motorB (spd, 0);
//    } else {
//      motorA(spd, 0);
//      motorB(spd, 0);
//    }
//  } else {
//    motorA(0, 0);
//    motorB(0, 0);
//    encoder1.numberTicks = 0;
//    encoder2.numberTicks = 0;
//    stateMotors = 0;
//  }
//}
//void backward(int dist) {
//  if (encoder1.numberTicks < dist) {
//    if ( encoder1.numberTicks > encoder2.numberTicks) {
//      motorA (spd, 1);
//      motorB (0, 1);
//    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
//      motorA (0, 1);
//      motorB (spd, 1);
//    } else {
//      motorA(spd, 1);
//      motorB(spd, 1);
//    }
//  } else {
//    motorA(0, 0);
//    motorB(0, 0);
//    encoder1.numberTicks = 0;
//    encoder2.numberTicks = 0;
//    stateMotors = 0;
//  }
//}
//
//void turn(int deg) {
//  int num = deg / 45;
//  if (num > 0) {
//    if (encoder1.numberTicks <= PULSE * num) {
//      if ( encoder1.numberTicks > encoder2.numberTicks) {
//        motorA (100, 1);
//        motorB (0, 0);
//      } else if ( encoder1.numberTicks < encoder2.numberTicks) {
//        motorA (0, 0);
//        motorB (100, 0);
//      } else {
//        motorA(100, 1);
//        motorB(100, 0);
//      }
//      //    Serial.printf("Motor 1: %d", encoder1.numberTicks);
//      //    Serial.printf("Motor 2: %d", encoder2.numberTicks);
//      //    Serial.println();
//    } else {
//      motorA(0, 0);
//      motorB(0, 0);
//      encoder1.numberTicks = 0;
//      encoder2.numberTicks = 0;
//      stateMotors = 0;
//    }
//  } else { //turn right
//    if (encoder1.numberTicks <= PULSE * num) {
//      if ( encoder1.numberTicks > encoder2.numberTicks) {
//        motorA (100, 0);
//        motorB (0, 1);
//      } else if ( encoder1.numberTicks < encoder2.numberTicks) {
//        motorA (0, 0);
//        motorB (100, 1);
//      } else {
//        motorA(100, 0);
//        motorB(100, 1);
//      }
//      //    Serial.printf("Motor 1: %d", encoder1.numberTicks);
//      //    Serial.printf("Motor 2: %d", encoder2.numberTicks);
//      //    Serial.println();
//    } else {
//      motorA(0, 0);
//      motorB(0, 0);
//      encoder1.numberTicks = 0;
//      encoder2.numberTicks = 0;
//      stateMotors = 0;
//    }
//  }
//}


void motorA ( int speed) {
  if (speed >= 0) {
    digitalWrite(DIRA, HIGH);
    ledcWrite(1, 255 - speed);
  }
  else {
    digitalWrite(DIRA, LOW);
    ledcWrite(1, speed);
  }
}

void motorB ( int speed) {
  if (speed >= 0) {
    digitalWrite(DIRB, HIGH);
    ledcWrite(2, 255 - speed);
  }
  else {
    digitalWrite(DIRB, LOW);
    ledcWrite(2, speed);
  }
}

void bnoSend() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= BNO055_SAMPLERATE_DELAY_MS) {
    // save the last time you came here
    previousMillis = currentMillis;

    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(euler.x());
    Serial.print(" Y: ");
    Serial.print(euler.y());
    Serial.print(" Z: ");
    Serial.print(euler.z());
    Serial.print("\t\t");

    /*
      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      Serial.print("qW: ");
      Serial.print(quat.w(), 4);
      Serial.print(" qX: ");
      Serial.print(quat.x(), 4);
      Serial.print(" qY: ");
      Serial.print(quat.y(), 4);
      Serial.print(" qZ: ");
      Serial.print(quat.z(), 4);
      Serial.print("\t\t");
    */

    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);

    //delay(BNO055_SAMPLERATE_DELAY_MS);

    //    if (euler.y() > 30) {     //this works
    //      msg = "MOVEFORWARD";
    //    } else if (euler.y() < -30) {
    //      msg = "MOVEBACKWARD";
    //    } else    if (euler.z() > 30) {
    //      msg = "MOVERIGHT";
    //    } else if (euler.z() < -30) {
    //      msg = "MOVELEFT";
    //    } else {
    //      msg = "";
    //    }

    msg = int(euler.y()) + String(",") + int(euler.z()) + "," + int(euler.x());
    Serial.println(msg);
    mesh.sendBroadcast( msg );
  }
}
