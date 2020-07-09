class Receiver
{
private:
    int _idx;
    int _pin_num;
    int _reading;

public:
    Receiver(int pin_num, int _idx);
    int get_idx();
    int get_pin();
    int get_reading();
    void receiver_init();
    void receiver_read();
}

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
}

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

    class ColAvo
    {
    private:
        Emitter _emitter;
        Receiver _receivers[5];

    public:
        ColAvo();
        void collision_avoidance_init();
        void collision_avoidance_main();
    }

    ColAvo::ColAvo() : _emitter(23)
    {
        _receivers[0] = Receiver(32, 0);
        _receivers[1] = Receiver(33, 1);
        _receivers[2] = Receiver(34, 2);
        _receivers[3] = Receiver(36, 3);
        _receivers[4] = Receiver(39, 4);
    }

    void ColAvo::collision_avoidance_init()
    {
        analogSetWidth(12);
        for (int i = 0; i < 5; i ++)
            _receivers[i].receiver_init();
        _emitter.emitter_init();
    }

    void ColAvo::collision_avoidance_main()
    {
        _emitter.emitter_control();
    }