//pins for motors
#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define PULSE 312

struct Tick {
  const uint8_t PIN;
  uint32_t numberTicks;
  bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void setup() {
  // put your setup code here, to run once:
  pinMode( PWMA, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  
  pinMode(encoder1.PIN, INPUT_PULLUP);
  pinMode(encoder2.PIN, INPUT_PULLUP);
  Serial.begin(115200);

}

void loop() {
  //Serial.println(".");
  //delay(1);
  // put your main code here, to run repeatedly:
  digitalWrite(PWMA,HIGH );
  digitalWrite(DIRA, LOW);
  digitalWrite(PWMB,HIGH );
  digitalWrite(DIRB, LOW);
  delay(2000);
    digitalWrite(PWMA, HIGH );
  digitalWrite(DIRA, LOW);
  digitalWrite(PWMB,LOW );
  digitalWrite(DIRB, HIGH);
    delay(2000);
  digitalWrite(PWMA,LOW );
  digitalWrite(DIRA, HIGH);
  digitalWrite(PWMB,LOW );
  digitalWrite(DIRB, HIGH);
  delay(2000);
  digitalWrite(PWMA,LOW );
  digitalWrite(DIRA, HIGH);
  digitalWrite(PWMB,HIGH );
  digitalWrite(DIRB, LOW);
  delay(2000);
}
