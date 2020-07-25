#define PWMA 27
#define DIRA 14
#define PWMB 12
#define DIRB 13
#define MUXI 34
#define MUXA 5
#define MUXB 18
#define MUXC 19
#define PULSE 5.69

int state = 1;
int res[8];
int angle;

struct Tick
{
    const uint8_t PIN;
    uint32_t numberTicks;
    bool tickOn;
};

Tick encoder1 = {16, 0, false};
Tick encoder2 = {17, 0, false};

void IRAM_ATTR isr()
{
    encoder1.numberTicks += 1;
    encoder1.tickOn = true;
}
void IRAM_ATTR isr2()
{
    encoder2.numberTicks += 1;
    encoder2.tickOn = true;
}

class Receiver
{
private:
    int _idx;
    int _pin_num;
    int _reading;

public:
    Receiver(int pin_num, int idx);
    int get_idx();
    int get_pin();
    int get_reading();
    void receiver_init();
    void receiver_read();
};

Receiver::Receiver(int pin_num, int idx)
{
    _idx = idx;
    _pin_num = pin_num;
}

int Receiver::get_idx()
{
    return _idx;
}

int Receiver::get_pin()
{
    return _pin_num;
}

int Receiver::get_reading()
{
    return _reading;
}

void Receiver::receiver_init()
{
    analogSetPinAttenuation(get_pin(), ADC_11db);
}

void Receiver::receiver_read()
{
    _reading = analogRead(get_pin());
}

class Emitter
{
private:
    int _em_tx;
    int _on_interval;
    int _off_interval;
    int _emitter_state;
    float _emitter_timer;

public:
    Emitter(int em_tx);
    void emitter_on();
    void emitter_off();
    void emitter_init();
    void emitter_control();
};

Emitter::Emitter(int em_tx)
{
    _em_tx = em_tx;
    _on_interval = 1;
    _off_interval = 20;
    _emitter_state = 0;
}

void Emitter::emitter_init()
{
    pinMode(_em_tx, OUTPUT);
    _emitter_timer = millis();
}

void Emitter::emitter_on()
{
    digitalWrite(_em_tx, HIGH);
}

void Emitter::emitter_off()
{
    digitalWrite(_em_tx, LOW);
}

void Emitter::emitter_control()
{
    if (_emitter_state) // emitter is turned on
    {
        float current = millis();
        if (current - _emitter_timer > _on_interval)
        {
            emitter_off();
            _emitter_timer = millis();
            _emitter_state = 0;
        }
    }
    else
    {
        float current = millis();
        if (current - _emitter_timer > _off_interval)
        {
            emitter_on();
            _emitter_timer = millis();
            _emitter_state = 1;
        }
    }
}

class Collision
{
private:
    Emitter _emitter;
    Receiver _receivers[5];
    int _receiver_state[5];
    int _receiver_readings[5];
    int _collision_state;
    int _collision_timer;

public:
    Collision(int pin_em);
    int *get_reading();
    int get_collision_timer();
    int get_collision_state();
    int decode_readings(int idx);
    int collision_avoidance_main();
    void update_readings();
    void emitter_control();
    void reset_collision_state();
    void reset_collision_timer();
    void collision_avoidance_init();
    void update_collision_timer(int num);
    void update_collision_state(int num);
};

Collision::Collision(int pin_em) : _emitter(23),
                                   _receivers{{32, 0}, {33, 1}, {34, 2}, {36, 3}, {39, 4}}
{
    reset_collision_timer();
    reset_collision_state();
    for (int i = 0; i < 5; i++)
        _receiver_readings[i] = 0;
}

int *Collision::get_reading()
{
    return _receiver_readings;
}

int Collision::decode_readings(int idx)
{
    if (_receiver_readings[idx] >= 3500)
        return 1;
    return 0;
}

void Collision::update_readings()
{
    for (int i = 0; i < 5; i++)
    {
        _receivers[i].receiver_read();
        _receiver_readings[i] = _receivers[i].get_reading();
    }
}

void Collision::collision_avoidance_init()
{
    analogSetWidth(12);
    for (int i = 0; i < 5; i++)
    {
        _receivers[i].receiver_init();
        _receiver_state[i] = 0;
    }
    _emitter.emitter_init();
}

int Collision::get_collision_state()
{
    return _collision_state;
}

int Collision::get_collision_timer()
{
    return _collision_timer;
}

void Collision::emitter_control()
{
    _emitter.emitter_control();
}

int Collision::collision_avoidance_main()
{
    // _emitter.emitter_control();
    update_readings();
    if (_receiver_readings[0] || _receiver_readings[1] || _receiver_readings[2] || _receiver_readings[3] || _receiver_readings[4])
    {
        if (decode_readings(2))
        // robot in front
        // initiate random timer to communicate
        {
            update_collision_timer(1000);
            update_collision_state(1);
            _receiver_state[2] = 1;
        }
        else if (decode_readings(4))
        // robot on the left
        {
            _receiver_state[4] = 1;
            if (_receiver_state[3])
                update_collision_state(0);
        }
    }
    else if (decode_readings(3))
    // robot on the right
    // this robot has priority to pass
    {
        _receiver_state[3] = 1;
        if (_receiver_state[4])
            update_collision_state(3);
    }
    else if (decode_readings(0) || decode_readings(1))
    {
        update_collision_state(0);
    }
    else
    {
        update_collision_state(0);
        for (int i = 0; i < 5; i++)
            _receiver_state[i] = 0;
    }
    // no robot blocking the way
    // clear to go
    return _collision_state;
}

void Collision::reset_collision_state()
{
    _collision_state = 0;
}

void Collision::reset_collision_timer()
{
    _collision_timer = 0;
}

void Collision::update_collision_state(int num)
{
    _collision_state = num;
}

void Collision::update_collision_timer(int num)
{
    _collision_timer = num;
}

// Emitter emitter(23);
// Receiver receiver(32, 0);
Collision collision(23);
int prev;

void setup()
{
    Serial.begin(115200);
    pinMode(PWMB, OUTPUT);
    pinMode(DIRB, OUTPUT);
    pinMode(DIRA, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(encoder1.PIN, INPUT_PULLUP);
    pinMode(encoder2.PIN, INPUT_PULLUP);
    attachInterrupt(encoder1.PIN, isr, FALLING);
    attachInterrupt(encoder2.PIN, isr2, FALLING);
    ledcAttachPin(PWMA, 1); // assign RGB led pins to channels
    ledcAttachPin(PWMB, 2);
    ledcSetup(1, 12000, 8); // 12 kHz PWM, 8-bit resolution
    ledcSetup(2, 12000, 8);
    collision.collision_avoidance_init();
}

void loop()
{
    collision.emitter_control();
    if (!collision.get_collision_timer())
    {
        int state = collision.collision_avoidance_main();
        Serial.println(state);
        if (!state)
        {
            digitalWrite(PWMA, HIGH);
            digitalWrite(DIRA, HIGH);
            digitalWrite(PWMB, HIGH);
            digitalWrite(DIRB, HIGH);
        }
        else
        {
            digitalWrite(PWMA, LOW);
            digitalWrite(DIRA, LOW);
            digitalWrite(PWMB, LOW);
            digitalWrite(DIRB, LOW);
            prev = millis();
        }
    }
    else
    {
        int current = millis();
        if (current - prev > collision.get_collision_timer())
        {
            prev = 0;
            collision.reset_collision_timer();
        }
    }

    // emitter.emitter_control();
    // receiver.receiver_read();
    // int reading = receiver.get_reading();
    // Serial.println(reading);
}

void forward(int dist)
{
    dist = distanceConvt(dist);

    if (encoder1.numberTicks < dist)
    {
        //    motorA(200, 0);
        //    motorB(200, 0);
        if (encoder1.numberTicks > encoder2.numberTicks)
        {
            motorA(200, 1);
            motorB(0, 1);
        }
        else if (encoder1.numberTicks < encoder2.numberTicks)
        {
            motorA(0, 1);
            motorB(200, 1);
        }
        else
        {
            motorA(200, 1);
            motorB(200, 1);
        }
        //    Serial.printf("Motor 1: %d", encoder1.numberTicks);
        //    Serial.printf("Motor 2: %d", encoder2.numberTicks);
        //    Serial.println("forward");
    }
    else
    {
        motorA(0, 0);
        motorB(0, 0);
        encoder1.numberTicks = 0;
        encoder2.numberTicks = 0;
        state = 0;
    }
}

void turnLeft(int deg)
{
    int num = deg;
    if (encoder1.numberTicks <= PULSE * num)
    {
        if (encoder1.numberTicks > encoder2.numberTicks)
        {
            motorA(100, 0);
            motorB(0, 0);
        }
        else if (encoder1.numberTicks < encoder2.numberTicks)
        {
            motorA(0, 0);
            motorB(100, 1);
        }
        else
        {
            motorA(100, 0);
            motorB(100, 1);
        }
    }
    else
    {
        motorA(0, 0);
        motorB(0, 0);
        encoder1.numberTicks = 0;
        encoder2.numberTicks = 0;
        state = 2;
    }
    Serial.printf("Motor 1: %d", encoder1.numberTicks);
    Serial.printf("Motor 2: %d", encoder2.numberTicks);
    Serial.println();
}

void motorA(int speed, int direction)
{
    if (direction)
    {
        digitalWrite(DIRA, HIGH);
        ledcWrite(1, 255 - speed);

        // sigmaDeltaWrite(0, 255 - speed);
    }
    else
    {
        digitalWrite(DIRA, LOW);
        //sigmaDeltaWrite(0, speed);
        ledcWrite(1, speed);
    }
}

void motorB(int speed, int direction)
{
    if (direction)
    {
        digitalWrite(DIRB, HIGH);
        //sigmaDeltaWrite(3, 255 - speed);
        ledcWrite(2, 255 - speed);
    }
    else
    {
        digitalWrite(DIRB, LOW);
        //sigmaDeltaWrite(3, speed);
        ledcWrite(2, speed);
    }
}

int distanceConvt(int dist)
{
    return dist * (3 / 4.1);
}
