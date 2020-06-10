#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 7
//SET ACCORDING TO THE ARUCO CODE

int* POS;
int* DST;
int ORI;
int TURN;
int DIST;
int STATE = 0;
const char* ID;

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

const char * ssid = "nowifi";
const char * password = "durf2020";

AsyncUDP udp;

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
  ID = "5";
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
      DynamicJsonDocument jInfo(1024);
      deserializeJson(jInfo, packet.data());
      const int Purpose = jInfo["Purpose"];
      switch (Purpose) {
        case 1:
          Serial.println("Position received");
          POS[0] = jInfo[ID][0][0];
          POS[1] = jInfo[ID][0][1];
          ORI = jInfo[ID][1];
          break;
        case 2:
          if (!STATE){
            Serial.println("Tasks received");
            STATE = 1;
            dstSelector(ORI, POS, DST, jInfo);
            break;
          } 
      }
      jInfo.clear();
    });
  }
  //  robot listening to tasks
//  if (udp.listenMulticast(IPAddress(224, 3, 29, 2), 10002)) {
//    udp.onPacket([](AsyncUDPPacket packet) {
//      if (!STATE) {
//        //        decode the packet only at idle STATE
//        STATE = 1;
//        DynamicJsonDocument jTask(1024);
//        deserializeJson(jTask, packet.data());
//        
//        Serial.println("Task received");
//      }
//    });
//  }
}

void loop() {
  if (STATE == 1) {
    forward(1000);
  } else if (STATE == 2) {
    turnLeft(TURN);
  } else if (STATE == 3) {
    turnRight(TURN);
  }
}

void dstSelector(int ORI, int* POS, int* DST, DynamicJsonDocument& jTask) {
  //  const int* dst = jTask["Task"];
  int x = jTask["Task"][0][0];
  int y = jTask["Task"][0][1];
  int idx = 0;
  int dist = (abs(x - POS[0]) + abs(y - POS[1]));
  const int len = jTask["Num"];
  for (int i = 0; i < len; i ++) {
    //  calc manhattan distance
    int x = jTask["Task"][i][0];
    int y = jTask["Task"][i][1];
    int new_dist = (abs(x - POS[0]) + abs(y - POS[1]));
    if (new_dist < dist) {
      dist = new_dist;
      idx = i;
    }
  }
  DST[0] = jTask["Task"][idx][0];
  DST[1] = jTask["Task"][idx][1];
  Serial.print("DST[0]: ");
  Serial.println(DST[0]);
  Serial.print("DST[1]: ");
  Serial.println(DST[1]);
  actionDecoder(ORI, POS, DST);
  jTask.clear();
}

void actionDecoder(int ORI, int* POS, int* DST) {
  //  calculate the robots' absolute position
  int x = DST[0] - POS[0];
  int y = DST[1] - POS[1];
  TURN = 90;
  if (abs(ORI) <= 5) {
    if (y >= 1) {
      STATE = 3;
      TURN = 180;
    } else if (y <= -1) {
      STATE = 1;
      DIST = y * 1000;
    } else {
      if (x >= 1) {
        STATE = 3;
      } else if (x <= -1) {
        STATE = 2;
      }
    }
  } else if (abs(ORI) >= 175) {
    if (y >= 1) {
      STATE = 1;
      DIST = abs(y) * 1000;
    } else if (y <= -1) {
      STATE = 3;
      TURN = 180;
    } else {
      if (x >= 1) {
        STATE = 2;
      } else if (x <= -1) {
        STATE = 3;
      }
    }
  } else if (ORI >= 85 && ORI <= 95) {
    if (x >= 1) {
      STATE = 1;
      DIST = x * 1000;
    } else if (x <= 1) {
      STATE = 3;
      TURN = 180;
    } else {
      if (y >= 1) {
        STATE = 2;
      } else if (y <= -1) {
        STATE = 3;
      }
    }
  } else if (ORI >= -95 && ORI <= -85) {
    if (x >= 1) {
      STATE = 3;
      TURN = 180;
    } else if (x <= -1) {
      STATE = 1;
      DIST = abs(x) * 1000;
    } else {
      if (y >= 1) {
        STATE = 3;
      } else if (y <= -1) {
        STATE = 2;
      }
    }
  } else {
    int remain = ORI % 90;
    if (abs(remain) >= 45) {
      STATE = (remain < 0) ? 2 : 3;
      TURN = 90 - abs(remain);
    } else {
      STATE = (remain > 0) ? 2 : 3;
      TURN = abs(remain);
    }
  }
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
    STATE = 0;
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
