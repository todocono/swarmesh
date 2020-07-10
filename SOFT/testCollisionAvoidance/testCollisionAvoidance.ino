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
    return _idx;
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
    int _receiver_readings[5];
    int _collision_state;
    int _collision_timer;

public:
    Collision(int pin_em);
    int get_collision_timer();
    int get_collision_state();
    int decode_readings(int idx);
    int collision_avoidance_main();
    void update_readings();
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

int Collision::decode_readings(int idx)
{
    if (_receiver_readings[idx] >= 2000)
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
        _receivers[i].receiver_init();
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

int Collision::collision_avoidance_main()
{
    _emitter.emitter_control();
    update_readings();
    if (decode_readings(2))
    // robot in front
    // initiate random timer to communicate
    {
        update_collision_timer(1);
        update_collision_state(1);
    }
    else if (decode_readings(3) || decode_readings(4))
    // robot on the right
    // this robot has priority to pass
    {
        update_collision_timer(10);
        update_collision_state(2);
    }
    else if (decode_readings(0) || decode_readings(1))
    // robot on the left
    {
        update_collision_state(3);
    }
    else
    {
        update_collision_state(0);
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

void setup()
{
}

void loop()
{
}