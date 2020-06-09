#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 7

int STATE = 0;
int* POS;
int* DST;
int ORI;

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

//void turnRight(int* POS, int* );

const char * ssid = "nowifi";
const char * password = "durf2020";

AsyncUDP udp;
DynamicJsonDocument jPos(1024);

void setup()
{
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMA, OUTPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  attachInterrupt(encoder1.PIN, isr, FALLING);
  attachInterrupt(encoder2.PIN, isr2, FALLING);
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);
  ledcSetup(1, 12000, 8);
  ledcSetup(2, 12000, 8);
  encoder1.numberTicks = 0;
  encoder2.numberTicks = 0;
  POS = (int*)malloc(sizeof(int) * 2);
  DST = (int*)malloc(sizeof(int) * 2);
  //  ORI = (int*)malloc(sizeof(int));
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
  //  listening to both task and current position on this channel
  if (udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      deserializeJson(jPos, packet.data());
      if (STATE == 0) {
        POS[0] = jPos["POS"][0][0];
        POS[1] = jPos["POS"][0][1];
        DST[0] = jPos["Task"][0];
        DST[1] = jPos["Task"][1];
        ORI = jPos["POS"][1];
        //        int ori = jPos["POS"][1];
        //        Serial.println(ORI);
        actionDecoder(POS, DST, ORI);
        Serial.println("Packet received");
      }
    });
  }
  //  robot listening to tasks
  //  if (udp.listenMulticast(IPAddress(224, 3, 29, 2), 10002)){
  //    udp.onPacket([](AsyncUDPPacket packet) {
  //      if (!STATE){
  ////        decode the packet at idle STATE
  //        STATE = 1;
  //        DynamicJsonDocument jTask(1024);
  //        deserializeJson(jTask, packet.data());
  //      }
  //    });
  //  }
}

void loop() {
  //  only calculate the path to the destination in STATE 1
  // and the robot knows its current location
  //  if (STATE == 1 && POS[0] >= 0){
  //    int* num = dstSelector(POS, jTask);
  //    int* dst = dstSelector(POS, jTask["Tasks"]);
  //  }
  if (STATE == 2) {
    turnLeft(ORI);
  } else if (STATE == 3) {
    turnRight(abs(ORI));
  }
}

void jsonHandlerPos(DynamicJsonDocument& jdoc) {
  const float rotation = jdoc["5"][1];
  const float x = jdoc["5"][0][0];
  const float y = jdoc["5"][0][1];
  Serial.print("x:");
  Serial.print(x);
  Serial.println();
  Serial.print("y");
  Serial.print(y);
  Serial.println();
  Serial.print("rotation: ");
  Serial.print(rotation);

}

//void dstSelector(int* POS, int* DST, DynamicJsonDocument& jTask){
////  const int* dst = jTask["Task"];
//  int len = jTask["Num"];
//  int idx = 0;
//  const float x = jTask["Task"][0][0];
//  const float y = jTask["Task"][0][1];
//  int dist = (abs(x - POS[0]) + abs(y - POS[1]));
//  for (int i = 0; i < len; i ++){
//////  calc manhattan distance
//    const float x = jTask["Task"][i][0];
//    const float y = jTask["Task"][i][1];
//    int new_dist = (abs(x - POS[0]) + abs(y - POS[1]));
//    if (new_dist < dist){
//      dist = new_dist;
//      idx = i;
//    }
//  }
//  DST[0] = jTask["Task"][idx][0];
//  DST[1] = jTask["Task"][idx][1];
//  jTask.clear();
//}

void actionDecoder(int* POS, int* DST, int ORI) {
  //  rotate to the right direction
  if (abs(ORI) >= 5) {
    if (ORI >= 0) {
      STATE = 2;
      turnLeft(ORI);
    } else {
      STATE = 3;
      turnRight(abs(ORI));
    }
  }
  //  } else if (abs(DST[1] - POS[1]) >= 1){
  //    int diff = DST[1] - POS[1];
  //    if (diff > 0){
  //      forward(1000);
  //    } else {
  //      turnLeft(180);
  //      forward(abs(diff));
  //    }
  //  } else if (abs(DST[0] - POS[0]) >= 1){
  //    int diff = DST[0] - POS[0];
  //    if (diff > 0){
  //      turnRight(90);
  //      forward(1000);
  //    } else {
  //      turnLeft(90);
  //      forward(1000);
  //    }
}

void forward(int dist) {
  if (encoder1.numberTicks < dist) {
    if ( encoder1.numberTicks > encoder2.numberTicks) {
      motorA (200, 0);
      motorB (0, 0);
    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
      motorA (0, 0);
      motorB (200, 0);
    } else {
      motorA(200, 0);
      motorB(200, 0);
    }
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    //    STATE = 0;
  }
}

void turnLeft(int deg) {
  if (encoder1.numberTicks <= PULSE * deg) {
    //    if ( encoder1.numberTicks > encoder2.numberTicks) {
    //      motorA (100, 0);
    //      motorB (0, 0);
    //    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
    //      motorA (0, 0);
    //      motorB (100, 1);
    //    } else {
    motorA(100, 0);
    motorB(100, 1);
    //    }
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    STATE = 0;
  }
}

void turnRight(int deg) {
  if (encoder1.numberTicks <= PULSE * deg) {
    //       if ( encoder1.numberTicks > encoder2.numberTicks) {
    //        motorA (100, 1);
    //        motorB (0, 0);
    //       } else if ( encoder1.numberTicks < encoder2.numberTicks){
    //        motorA (0, 0);
    //        motorB (100, 0);
    //      } else {
    motorA(100, 1);
    motorB(100, 0);
    //      }
  } else {
    motorA(0, 0);
    motorB(0, 0);
    encoder1.numberTicks = 0;
    encoder2.numberTicks = 0;
    STATE = 0;
  }
}

void motorA ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRA, HIGH);
    ledcWrite(1, 255 - speed);

    // sigmaDeltaWrite(0, 255 - speed);
  }
  else {
    digitalWrite(DIRA, LOW);
    //sigmaDeltaWrite(0, speed);
    ledcWrite(1, speed);
  }
}

void motorB ( int speed, int direction) {
  if (direction) {
    digitalWrite(DIRB, HIGH);
    //sigmaDeltaWrite(3, 255 - speed);
    ledcWrite(2, 255 - speed);
  }
  else {
    digitalWrite(DIRB, LOW);
    //sigmaDeltaWrite(3, speed);
    ledcWrite(2, speed);
  }
}
