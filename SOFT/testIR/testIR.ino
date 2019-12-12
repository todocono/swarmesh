// NYU Shanghai
// This code is part of the research group SWARMESH
// https://hackaday.io/project/165119-nyu-shanghai-swarm-robot-system
// 

#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define MUXI 34
#define MUXA 5
#define MUXB 18
#define MUXC 19
#define IRT  23

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


  pinMode( IRT, OUTPUT);
  
  ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
  ledcAttachPin(PWMB, 2);
  ledcAttachPin(IRT, 3);
  // Initialize channels
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);
  ledcSetup(3, 12000, 8);



  ledcWrite(3, 128);

  adcAttachPin(MUXI);
  analogSetWidth(12);
  analogSetPinAttenuation(34, ADC_11db);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  // a for loop to
  digitalWrite(IRT, LOW);
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
    Serial.print("\t");
    delay(10);
  }
  Serial.println();
  delay(1);
}
