#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define MUXI 34
#define MUXA 5
#define MUXB 18
#define MUXC 19
#define PULSE 312

int res[8];

void setup() {
  // put your setup code here, to run once:
  pinMode( MUXI, INPUT);
  pinMode( MUXA, OUTPUT);
  pinMode( MUXB, OUTPUT);
  pinMode( MUXC, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMA, OUTPUT);
  pinMode( 23, OUTPUT);
  adcAttachPin(MUXI);
  analogSetWidth(12);
  analogSetPinAttenuation(34, ADC_11db);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(23, HIGH);
  delay(1000);
  digitalWrite(23, LOW);
  delay(1000);
}
