class Motor
{
private:
    int _PWM;
    int _DIR;
    int _IDX;
    uint8_t _PIN;
    uint32_t _number_ticks;
    bool _tick_on;
    void _reset_encoder();

public:
    Motor(uint8_t PIN, int PWM, int DIR, int IDX);
    void motor_move(int spd, int dir);
    void motor_stop();
    void IRAM_ATTR isr();
    const int get_tick();
    const int get_idx();
    const int get_pwm();
    const int get_dir();
    const uint8_t get_pin();
};

Motor::Motor(uint8_t PIN, int PWM, int DIR, int IDX)
{
    _PIN = PIN;
    _PWM = PWM;
    _DIR = DIR;
    _IDX = IDX;
    _number_ticks = 0;
    _tick_on = false;
}

void Motor::IRAM_ATTR isr()
{
    _number_ticks++;
    _tick_on = true;
}

void Motor::motor_move(int spd, int dir)
{
    if (dir)
    {
        digitalWrite(_DIR, HIGH);
        ledcWrite(get_idx(), 255 - spd);
    }
    else
    {
        digitalWrite(_DIR, LOW);
        ledcWrite(get_idx(), spd);
    }
}

void Motor::_reset_encoder()
{
    _number_ticks = 0;
}

void Motor::motor_stop()
{
    _reset_encoder();
    motor_move(0, 0);
}

const int Motor::get_tick()
{
    return _number_ticks;
}

const int Motor::get_idx()
{
    return _IDX;
}

const int Motor::get_pwm()
{
    return _PWM;
}

const int Motor::get_dir()
{
    return _DIR;
}

const uint8_t Motor::get_pin()
{
    return _PIN;
}

class Locomotion : public Motor
{
private:
    Motor _motor1;
    Motor _motor2;
    Motor *_motors;
    int _PULSE;
    void _forward(int dist);
    void _turn(int deg, char dir);

public:
    Locomotion(int pin1, int pin2);
    void motor_init();
}

Locomotion::Locomotion(int pin1, int pin2)
{
    _motor1(pin1, 27, 14, 1);
    _motor2(pin2, 12, 13, 2);
    _PULSE = 7;
}

void Locomotion::_forward(int dist)
{
    if (_motor1.get_tick() < dist)
    {
        if (_motor1.get_tick() > _motor2.get_tick())
        {
            _motor1.motor_move(200, 0);
            _motor2.motor_move(0, 0);
        }
        else if (_motor1.get_tick() < _motor2.get_tick())
        {
            _motor1.motor_move(0, 0);
            _motor2.motor_move(200, 0);
        }
        else
        {
            _motor1.motor_move(200, 0);
            _motor2.motor_move(200, 0);
        }
    }
    else
    {
        for (int i = 0; i < 2; i++) _motors[i].motor_stop();
        // _motor2.motor_move(0, 0);
        // STATE = 0;
    }
}

void Locomotion::_turn(int deg, char dir)
{
    int num1;
    int num2;
    switch (dir)
    {
    case 'L':
        num1 = 0;
        num2 = 1;
        break;
    
    case 'R':
        num1 = 1;
        num2 = 0;
        break;
    }
    if (_motor1.get_tick() <= PULSE * deg)
    {
        //    if ( encoder1.numberTicks > encoder2.numberTicks) {
        //      motorA (100, 0);
        //      motorB (0, 0);
        //    } else if ( encoder1.numberTicks < encoder2.numberTicks) {
        //      motorA (0, 0);
        //      motorB (100, 1);
        //    } else {
        _motor1.motor_move(100, num1);
        _motor2.motor_move(100, num2);
        //    }
    }
    else
    {
        for (int i = 0; i < 2; i++) _motors[i].motor_stop();
        // STATE = 0;
    }
}

void Locomotion::motor_init()
{
    _motors = (Motor *)malloc(sizeof(Motor) * 2);
    _motors[0] = _motor1;
    _motors[1] = _motor2;
    for (int i = 0; i < 2; i++)
    {
        pinMode(_motors[i].get_pwm(), OUTPUT);
        pinMode(_motors[i].get_dir(), OUTPUT);
        pinMode(_motors[i].get_pin(), INPUT_PULLUP);
        attachInterrupt(_motors[i].get_pin(), _motors[i].isr(), FALLING);
        ledcAttachPin(_motors[i].get_pwm(), _motors[i].get_idx());
        ledcSetup(_motors[i].get_idx(), 12000, 8);
    }
}