
#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define MUXI 34
#define MUXA 5
#define MUXB 18
#define MUXC 19

void setup() {
  // put your setup code here, to run once:
  // set up the adc function
  pinMode( MUXI, INPUT);
  pinMode( MUXA, OUTPUT);
  pinMode( MUXB, OUTPUT);
  pinMode( MUXC, OUTPUT);
  pinMode( PWMB, OUTPUT);
  pinMode( DIRB, OUTPUT);
  pinMode( DIRA, OUTPUT);
  pinMode( PWMA, OUTPUT);

  adcAttachPin(MUXI);
  analogSetWidth(12);
  analogSetPinAttenuation(34, ADC_11db);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  // a for loop to
  int val[8];
  for (int i = 0; i < 8; i ++) {
    int pin1 = i & 0x001;
    int pin2 = (i >> 1) & 0x001;
    int pin3 = (i >> 2) & 0x001;
    digitalWrite( MUXA, pin1);
    digitalWrite( MUXB, pin2);
    digitalWrite( MUXC, pin3);
    delay(10);
    int res = analogRead(MUXI);
    val[i] = res;
    Serial.print(res );
    Serial.print(" ");
    delay(10);
  }
  Serial.println();
  delay(1);
}
