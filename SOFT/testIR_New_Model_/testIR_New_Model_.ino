#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define IR1 32
#define IR2 33
#define IR3 34
#define IR4 36
#define IR5 39

int STATE = 0;
int prev_time;

void setup()
{
  // put your setup code here, to run once:
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(23, OUTPUT);
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);
  ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
  ledcSetup(2, 12000, 8);
  analogSetWidth(12);
  analogSetPinAttenuation(IR1, ADC_11db);
  analogSetPinAttenuation(IR2, ADC_11db);
  analogSetPinAttenuation(IR3, ADC_11db);
  analogSetPinAttenuation(IR4, ADC_11db);
  analogSetPinAttenuation(IR5, ADC_11db);
  Serial.begin(115200);
  prev_time = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  int current = millis();
  if (STATE && current - prev_time > 1)
  {
    prev_time = millis();
    digitalWrite(23, LOW);
  }
  else if (!STATE && current - prev_time > 20)
  {
    prev_time = millis();
    digitalWrite(23, HIGH);
  }
  int reading1 = analogRead(IR1);
  int reading2 = analogRead(IR2);
  int reading3 = analogRead(IR3);
  int reading4 = analogRead(IR4);
  int reading5 = analogRead(IR5);
  if (reading1 || reading2 || reading3 || reading4 || reading5)
  {
    Serial.print("Reading1: ");
    Serial.println(reading1);
    Serial.print("Reading2: ");
    Serial.println(reading2);
    Serial.print("Reading3: ");
    Serial.println(reading3);
    Serial.print("Reading4: ");
    Serial.println(reading4);
    Serial.print("Reading5: ");
    Serial.println(reading5);
    Serial.println();
    delay(100);
  }
}
