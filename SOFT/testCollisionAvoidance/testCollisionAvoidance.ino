#include <ArduinoJson.h>
#include "WiFi.h"
#include "AsyncUDP.h"

struct Encoder
{
    const uint8_t PIN;
    uint32_t numberTicks;
    bool tickOn;
};

Encoder encoder1 = {16, 0, false};
Encoder encoder2 = {17, 0, false};

void IRAM_ATTR isr1()
{
    encoder1.numberTicks++;
    encoder1.tickOn = true;
}

void IRAM_ATTR isr2()
{
    encoder2.numberTicks++;
    encoder2.tickOn = true;
}

class Motor
{
private:
    Encoder *_encoder;
    int _PWM;
    int _DIR;
    int _IDX;
    uint8_t _PIN;

public:
    Motor(Encoder *encoder, int PWM, int DIR, int IDX);
    void motor_move(int spd, int dir);
    void motor_stop();
    void reset_encoder();
    const int get_idx();
    const int get_pwm();
    const int get_dir();
    const int get_tick();
    const uint8_t get_pin();
};

Motor::Motor(Encoder *encoder, int PWM, int DIR, int IDX)
{
    _PIN = encoder->PIN;
    _PWM = PWM;
    _DIR = DIR;
    _IDX = IDX;
    _encoder = encoder;
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

void Motor::reset_encoder()
{
    (*_encoder).numberTicks = 0;
}

void Motor::motor_stop()
{
    reset_encoder();
    motor_move(0, 0);
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

const int Motor::get_tick()
{
    return (*_encoder).numberTicks;
}

const uint8_t Motor::get_pin()
{
    return _PIN;
}

class PID
{
private:
    unsigned long lastTime;
    double Input, Output, Setpoint;
    double errSum, lastErr;
    double kp, ki, kd;
    int SampleTime;

public:
    PID();
    int pid_compute(int error);
    void pid_tuning(double Kp, double Ki, double Kd);
};

PID::PID()
{
    kp = 0.015;
    ki = 0.0;
    kd = 0.0;
    errSum = 0;
    lastErr = 0;
    SampleTime = 1000;
}

int PID::pid_compute(int error)
{
    Output = kp * error; // + ki * errSum + kd * dErr;
    // unsigned long now = millis();
    // int timeChange = (now - lastTime);
    // if (timeChange >= SampleTime)
    // {
    //     /*Compute all the working error variables*/
    //     errSum += error;
    //     double dErr = (error - lastErr);

    //     /*Compute PID Output*/
    //     Output = kp * error + ki * errSum + kd * dErr;

    //     /*Remember some variables for next time*/
    //     lastErr = error;
    //     lastTime = now;
    // }
    return Output;
}

void PID::pid_tuning(double Kp, double Ki, double Kd)
{
    double SampleTimeInSec = ((double)SampleTime) / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
}

class Locomotion
{
private:
    PID _pid;
    Motor _motor1;
    Motor _motor2;
    Motor *_motors;
    float _PULSE;
    int _width;
    int _off_course;
    int _prev_tick[2];
    int _calc_ori(int ori);
    int _calc_error(int *pos, const char *traj);
    int _tick_cvt(int dist);

public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);
    int pos[3];
    int forward(int dist, int error);
    int turn(int deg, char dir);
    void tune_pid(double kp);
    void reset_encoders();
    void motor_init();
};

Locomotion::Locomotion(Encoder *encoder1, Encoder *encoder2) : _motor2(encoder1, 27, 14, 1),
                                                               _motor1(encoder2, 12, 13, 2)
{
    _PULSE = 5.69;
    _off_course = 0;
    for (int i = 0; i < 2; i++)
        _prev_tick[i] = 0;
}

void Locomotion::tune_pid(double kp)
{
}

int Locomotion::_tick_cvt(int dist)
{
    return dist * (3 / 4.1);
}

int Locomotion::forward(int dist, int error)
{
    dist = _tick_cvt(dist);
    int spd = 220;
    _off_course = 0;
    if (abs(error) >= 100)
        _off_course = 1;
    else
        _off_course = 0;
    int tick1 = _motor1.get_tick();
    int tick2 = _motor2.get_tick();
    if (tick1 < dist)
    {
        if ((dist - tick1) < 750)
            spd = map(dist - tick1, 0, 750, 150, 220);
        int gap = 0;
        if (!_off_course)
            gap = -(tick1 - tick2) * 0.5;
        else
            gap = _pid.pid_compute(error);
        _motor1.motor_move(spd - gap, 1);
        _motor2.motor_move(spd + gap, 1);
        int tick1 = _motor1.get_tick();
        int ori = pos[2];
        if (abs(ori) <= 5)
            pos[1] -= tick1 - _prev_tick[0];
        else if (abs(ori) >= 175)
            pos[1] += tick1 - _prev_tick[0];
        else if (ori <= 95 && ori >= 85)
            pos[0] += tick1 - _prev_tick[0];
        else if (ori <= -85 && ori >= -95)
            pos[0] -= tick1 - _prev_tick[0];
        _prev_tick[0] = tick1;
        return 0;
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            _motors[i].motor_stop();
            _prev_tick[i] = 0;
        }
        return 1;
    }
}

int Locomotion::turn(int deg, char dir)
{
    int num1 = (dir == 'L') ? 0 : 1;
    int num2 = (dir == 'L') ? 1 : 0;
    int tick = _motor1.get_tick();
    if (tick <= _PULSE * deg)
    {
        int gap = 0.1 * (_PULSE * deg - tick);
        _motor1.motor_move(120 + gap, num1);
        _motor2.motor_move(120 + gap, num2);
        // int tick = _motor1.get_tick();
        // _prev_tick[0] = tick;
        return 0;
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            _motors[i].motor_stop();
            _prev_tick[i] = 0;
        }
        switch (dir)
        {
        case 'L':
            pos[2] -= deg;
            if (pos[2] < -180)
                pos[2] += 360;
            break;
        case 'R':
            pos[2] += deg;
            if (pos[2] > 180)
                pos[2] -= 360;
        }
        return 1;
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
        //    attachInterrupt(_motors[i].get_pin(), _motors[i].isr(), FALLING);
        if (i)
            attachInterrupt(_motors[i].get_pin(), isr1, FALLING);
        else
            attachInterrupt(_motors[i].get_pin(), isr2, FALLING);
        ledcAttachPin(_motors[i].get_pwm(), _motors[i].get_idx());
        ledcSetup(_motors[i].get_idx(), 12000, 8);
    }
}

void Locomotion::reset_encoders()
{
    for (int i = 0; i < 2; i++)
    {
        _motors[i].reset_encoder();
        _prev_tick[i] = 0;
    }
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
    _on_interval = 5;
    _off_interval = 10;
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
    int _collision_coordinates[2];
    int _collision_start_time;
    int _collision_hold_timer;
    int _collision_action;
    int _collision_state;

public:
    Collision(int pin_em);
    int *get_reading();
    int get_collision_state();
    int get_collision_action();
    int decode_readings(int idx);
    int get_collision_start_time();
    int get_collision_hold_timer();
    int collision_avoidance_main();
    void update_readings();
    void emitter_control();
    void collision_avoidance_init();
    void set_collision_start_time();
    void update_collision_state(int num);
    void update_collision_action(int action);
    void update_collision_hold_timer(int num);
};

Collision::Collision(int pin_em) : _emitter(23),
                                   _receivers{{32, 0}, {33, 1}, {34, 2}, {36, 3}, {39, 4}}
{

    for (int i = 0; i < 5; i++)
        _receiver_readings[i] = 0;
}

int *Collision::get_reading()
{
    return _receiver_readings;
}

int Collision::get_collision_action()
{
    return _collision_action;
}

int Collision::get_collision_start_time()
{
    return _collision_start_time;
}

int Collision::decode_readings(int idx)
{
    if (_receiver_readings[idx] >= 3500)
        return 1;
    return 0;
}

void Collision::update_collision_action(int action)
{
    _collision_action = action;
}

void Collision::set_collision_start_time()
{
    _collision_start_time = millis();
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

int Collision::get_collision_hold_timer()
{
    return _collision_hold_timer;
}

void Collision::emitter_control()
{
    _emitter.emitter_control();
}

int Collision::collision_avoidance_main()
{
    // _emitter.emitter_control();
    update_readings();
    if (_receiver_readings[0] || _receiver_readings[1] || _receiver_readings[2] > 400 || _receiver_readings[3] || _receiver_readings[4])
    {
        if (decode_readings(2))
        {
            update_collision_state(1);
        }
        else if (decode_readings(3) || decode_readings(4))
        {
            update_collision_state(3);
            set_collision_start_time();
            update_collision_hold_timer(1000);
        }
        else if (decode_readings(0) || decode_readings(1))
        {
            update_collision_state(2);
        }
        else
        {
            update_collision_state(0);
            update_collision_hold_timer(0);
            for (int i = 0; i < 5; i++)
                _receiver_state[i] = 0;
        }
    }
    return _collision_state;
}

void Collision::update_collision_state(int num)
{
    _collision_state = num;
}

void Collision::update_collision_hold_timer(int num)
{
    _collision_hold_timer = num;
}

class Communication
{
private:
    char *_ssid;
    char *_password;

public:
    Communication(char *ssid, char *password);
    void wifi_init();
    // char *json_creator(int *POS);
    // void udp_init(Destinations dst, const char *ID, int *POS, int *ORI, DynamicJsonDocument &jInfo);
    void collision_com_init(char *str, int *route, int *pos, char *id);
    void collision_com_reply(char *str, int *pos, int action, char id);
};

Communication::Communication(char *ssid, char *password)
{
    _ssid = ssid;
    _password = password;
}

void Communication::wifi_init()
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("WiFi Failed");
        while (1)
        {
            delay(1000);
        }
    }
}

void Communication::collision_com_init(char *str, int *route, int *pos, char *id)
{
    // char *jsonStr = (char *)malloc(sizeof(char) * 80);
    char jsonStr[80];
    const size_t capacity = 2 * JSON_ARRAY_SIZE(2) + JSON_OBJECT_SIZE(4);
    DynamicJsonDocument doc(1024);
    doc["Purpose"] = 4;
    doc["ID"] = 5;
    JsonArray now_route = doc.createNestedArray("route");
    JsonArray now_pos = doc.createNestedArray("pos");
    for (int i = 0; i < 2; i++)
    {
        now_route.add(route[i]);
        now_pos.add(pos[i]);
    }
    serializeJson(doc, jsonStr);
    doc.clear();
    free(pos);
    for (int i = 0; i < 80; i++)
        str[i] = jsonStr[i];
}

void Communication::collision_com_reply(char *str, int *pos, int action, char id)
{
    char jsonStr[80];
    DynamicJsonDocument doc(1024);
    doc["Purpose"] = 5;
    doc["action"] = action;
    doc["ID"] = id;
    serializeJson(doc, jsonStr);
    doc.clear();
    free(pos);
    for (int i = 0; i < 80; i ++)
        str[i] = jsonStr[i];
}

class Robot
{
private:
    int _ctr;
    int _ptr;
    int _turn;
    int _dist;
    int _error;
    int _STATE;
    int _width;
    int _prev_time;
    int _task_size;
    int _turn_timer;
    int _turn_update;
    int _collision_com_state;
    int _collision_com_timer;
    int _collision_com_start_time;
    int *_pos;
    int _dst[2];
    int **_route;
    char _direction;
    char *_ID;
    Locomotion _loc;
    Collision _collision;
    Communication _communication;

public:
    Robot(Encoder *encoder1, Encoder *encoder2, char *ssid, char *password);
    char *get_id();
    int get_size();
    int get_state();
    int get_error();
    int check_collision();
    int get_collision_state();
    int get_collision_com_state();
    int get_collision_com_timer();
    int get_collision_com_start_time();
    int collision_distance_calculator(int *pos, int *coordinate, char *direction);
    int verify_collision_coordinate(int *pos, int *route, int *coordinate, char *direction);
    int *get_pos();
    int *process_route(int *pos, int *route);
    int *collision_action(int *pos, int *coordinate, char *direction);
    int **get_route();
    void robot_init();
    void calc_error();
    void check_task();
    void auto_route();
    void update_est();
    void main_executor();
    void action_decoder();
    void reroute(char dir);
    void tune_pid(double kp);
    void update_abs(int *pos);
    void update_state(int state);
    void update_collision_com_start_time();
    void update_collision_com_timer(int time);
    void update_collision_com_state(int state);
    void trajectory_calculator(char *axis, int *value, int *pos, int *route);
    void collision_coordinate_calculator(char *direction1, char *direction2, int value1, int value2, int *coordinate);
    void collision_com_init(char *jsonStr);
    void collision_com_reply(char *jsonStr, char id, int action);
};

Robot::Robot(Encoder *encoder1, Encoder *encoder2, char *ssid, char *password) : _loc(encoder1, encoder2),
                                                                                 _communication(ssid, password),
                                                                                 _collision(23)
{
    _ctr = 0;
    _ptr = 1;
    _STATE = 0;
    _width = 8;
    _prev_time = 0;
    _turn_update = 0;
    _turn_timer = 800;
    _collision_com_state = 0;
    _pos = _loc.pos;
}

int Robot::get_collision_com_state()
{
    return _collision_com_state;
}

int Robot::get_collision_com_timer()
{
    return _collision_com_timer;
}

int Robot::get_collision_com_start_time()
{
    return _collision_com_start_time;
}

int Robot::get_collision_state()
{
    return _collision.get_collision_state();
}

void Robot::update_collision_com_start_time()
{
    _collision_com_start_time = millis();
}

int Robot::get_error()
{
    return _error;
}

int *Robot::collision_action(int *pos, int *coordinate, char *direction)
{
    int self_distance, other_distance;
    self_distance = collision_distance_calculator(_pos, coordinate, &_direction);
    other_distance = collision_distance_calculator(pos, coordinate, direction);
    int *action = (int *)malloc(sizeof(int) * 2);
    if (self_distance < other_distance)
    {
        action[0] = 1;
        action[1] = 0;
    }
    else
    {
        action[0] = 0;
        action[1] = 1;
    }
    // 0 means stop
    // 1 means keep moving
    _collision.update_collision_action(action[0]);
    return action;
}

int Robot::collision_distance_calculator(int *pos, int *coordinate, char *direction)
{
    // needs modification
    int distance, gap1, gap2;
    gap1 = coordinate[0] - _pos[0];
    gap2 = coordinate[1] - _pos[1];
    int *action = (int *)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
        action[i] = 1;
    switch (_direction)
    {
    case 'N':
        distance = -gap2;
    case 'S':
        distance = gap2;
    case 'W':
        distance = -gap1;
    case 'E':
        distance = gap1;
    }
    return distance;
}

int Robot::verify_collision_coordinate(int *pos, int *route, int *coordinate, char *direction)
{
    int x_diff, y_diff;
    x_diff = coordinate[0] - pos[0];
    y_diff = coordinate[1] - pos[1];
    switch (*direction)
    {
    case 'N':
        if (-y_diff <= 3000 && coordinate[1] > route[1])
            return 1;
        else
            return 0;
    case 'S':
        if (y_diff <= 3000 && coordinate[1] < route[1])
            return 1;
        else
            return 0;
    case 'W':
        if (-x_diff <= 3000 && coordinate[0] > route[0])
            return 1;
        else
            return 0;
    case 'E':
        if (x_diff <= 3000 && coordinate[0] < route[0])
            return 1;
        else
            return 0;
    }
}

int *Robot::process_route(int *pos, int *route)
{
    // compare the other robot's route with its own route and see whether the routes have conflicts
    // return 1 if the routes have conflicts
    // return 0 otherwise
    char direction1, direction2;
    int value1, value2;
    int *coordinate = (int *)malloc(sizeof(int) * 2);
    for (int i = 0; i < 2; i++)
        coordinate[i] = -1;
    trajectory_calculator(&direction1, &value1, _pos, _route[_ptr]);
    trajectory_calculator(&direction2, &value2, pos, route);
    Serial.print("Self trajectory: ");
    Serial.println(direction1);
    Serial.print("Other trajectory: ");
    Serial.println(direction2);
    Serial.println();
    // two robots are parallel to each other
    if (direction1 == 'N' || direction1 == 'S')
    {
        if (direction2 == 'N' || direction2 == 'S')
            return coordinate;
    }
    else
    {
        if (direction2 == 'W' || direction2 == 'E')
            return coordinate;
    }
    collision_coordinate_calculator(&direction1, &direction2, value1, value2, coordinate);
    if (verify_collision_coordinate(_pos, _route[_ptr], coordinate, &direction1) && verify_collision_coordinate(pos, route, coordinate, &direction2))
    {
        return coordinate;
    }
    else
    {
        for (int i = 0; i < 2; i++)
            coordinate[i] = -1;
        return coordinate;
    }
}

int *Robot::get_pos()
{
    int *ptr = (int *)malloc(sizeof(int) * 3);
    for (int i = 0; i < 3; i++)
        ptr[i] = _pos[i];
    return ptr;
}

char *Robot::get_id()
{
    return _ID;
}

int Robot::get_size()
{
    return _task_size;
}

int Robot::get_state()
{
    return _STATE;
}

void Robot::robot_init()
{
    _loc.motor_init();
    _collision.collision_avoidance_init();
}

void Robot::calc_error()
{
    int *pos = get_pos();
    int *crt_task = _route[_ptr];
    float error1 = pos[0] - crt_task[0];
    float error2 = pos[1] - crt_task[1];
    switch (_direction)
    {
    case 'N':
        _error = error1;
        break;
    case 'S':
        _error = -error1;
        break;
    case 'W':
        _error = -error2;
        break;
    case 'E':
        _error = error2;
        break;
    }
    free(pos);
}

void Robot::auto_route()
{
    int tmp;
    int dst[2];
    dst[0] = 3000;
    dst[1] = 3000;
    int x_distance = (dst[0] - _pos[0]) / 1500;
    int y_distance = (dst[1] - _pos[1]) / 1500;

    // get the remainder of the division
    int x_remainder = (dst[0] - _pos[0]) % 1500;
    int y_remainder = (dst[1] - _pos[1]) % 1500;
    if (abs(x_remainder) > 50 && abs(y_remainder) > 50)
        tmp = 2;
    else
        tmp = 0;
    if (x_distance && y_distance)
    {
        // when both x_distance and y_distance is greater than 0
        srand((unsigned)time(0));
        int random_num = rand() % (abs(x_distance) + 1);
        random_num = (x_distance > 0) ? random_num : (-random_num);
        _task_size = tmp + 4;
        _route = (int **)malloc(sizeof(int *) * _task_size);
        for (int i = 0; i < _task_size; i++)
            _route[i] = (int *)malloc(sizeof(int) * 2);
        Serial.println("distance calculated");
        _route[tmp + 0][0] = dst[0] - x_distance * 1500;
        _route[tmp + 0][1] = dst[1] - y_distance * 1500;
        _route[tmp + 1][0] = dst[0] - random_num * 1500;
        _route[tmp + 1][1] = dst[1] - y_distance * 1500;
        _route[tmp + 2][0] = dst[0] - random_num * 1500;
        _route[tmp + 2][1] = dst[1];
        _route[tmp + 3][0] = dst[0];
        _route[tmp + 3][1] = dst[1];
    }
    else
    {
        _task_size = tmp + 2;
        _route = (int **)malloc(sizeof(int *) * _task_size);
        for (int i = 0; i < 2; i++)
            _route[i] = (int *)malloc(sizeof(int) * 2);
        for (int i = 0; i < 2; i++)
            _route[tmp + 1][i] = dst[i];
        if (x_distance)
        {
            _route[tmp][0] = dst[0] - x_distance * 1500;
            _route[tmp][1] = dst[1];
        }
        else
        {
            _route[tmp][0] = dst[0];
            _route[tmp][1] = dst[1] - y_distance * 1500;
        }
    }
    if (tmp)
    {
        for (int i = 0; i < 2; i++)
            _route[0][i] = _pos[i];
        _route[1][0] = _pos[0] + x_remainder;
        _route[1][1] = _pos[1];
    }
}

void Robot::tune_pid(double kp)
{
    _loc.tune_pid(kp);
}

void Robot::update_state(int state)
{
    _STATE = state;
}

void Robot::update_collision_com_state(int state)
{
    _collision_com_state = state;
}

void Robot::trajectory_calculator(char *direction, int *value, int *pos, int *route)
{
    // assuming the robot is completely on track
    if (!(pos[0] - route[0]))
    {
        if (route[1] - pos[1] > 0)
            *direction = 'S';
        else
            *direction = 'N';
        *value = route[0];
    }
    else
    {
        if (route[0] - pos[0] > 0)
            *direction = 'E';
        else
            *direction = 'W';
        *value = route[1];
    }
}

void Robot::collision_coordinate_calculator(char *direction1, char *direction2, int value1, int value2, int *coordinate)
{
    if (*direction1 == 'S' || *direction1 == 'N')
    {
        coordinate[0] = value1;
        coordinate[1] = value2;
    }
    else
    {
        coordinate[0] = value2;
        coordinate[1] = value1;
    }
}

void Robot::update_abs(int *pos)
{
    // calculate the difference between the estimate and the actual
    // compensate for the difference using the _dist variable
    if (_STATE == 1)
    {
        _loc.reset_encoders();
        if (_direction == 'N' || _direction == 'S')
            _dist = abs(_route[_ptr][1] - pos[1]) - 150;
        else
            _dist = abs(_route[_ptr][0] - pos[0]) - 150;
        _dist = (_dist < 0) ? 0 : _dist;
    }
    if (_STATE != 2 && _STATE != 3 && (millis() - _prev_time) >= _turn_timer)
    {
        int turn_error = abs(pos[2] - _pos[2]);
        if (turn_error > 20)
        {
            // receive the orientation again
            if (_ctr)
            {
                // already received the absurd orientation
                // accept
                _pos[2] = pos[2];
                _prev_time = millis();
                _turn_update = 0;
            }
            else
            {
                // wait for another interval
                _prev_time = millis();
                _turn_update = 0;
                _ctr++;
            }
        }
        else
        {
            _pos[2] = pos[2];
            _prev_time = 0;
            _turn_update = 1;
        }
    }
    for (int i = 0; i < 2; i++)
        _pos[i] = pos[i];
}

int **Robot::get_route()
{
    return _route;
}

int Robot::check_collision()
{
    // there is something in the view field of the robot
    // the robot is in forward state
    _collision.emitter_control();
    // && get_state() == 1
    //  && !_collision_com_state
    if (millis() - _collision.get_collision_start_time() > _collision.get_collision_hold_timer() && _collision.collision_avoidance_main())
    {
        Serial.print("Collision state: ");
        Serial.println(_collision.get_collision_state());
        switch (_collision.get_collision_state())
        {
        case 1:
            // vehicle in front
            // still needs to confirm this
            // chances are the robot would run out of the arena
            // reroute(_direction);
            update_state(0);
            break;
        case 2:
            // vehicle on the left
            // no communication needed
            // either parallel to each other or the other robot would initiate communication

            // srand((unsigned)time(0));
            // _collision_timer = rand(300);
            // return 1;
            break;
        case 3:
            // vehicle on the right
            srand((unsigned)time(0));
            update_collision_com_timer((rand() % 300));
            update_collision_com_start_time();
            Serial.print("Collision timer set: ");
            Serial.println(get_collision_com_timer());
            return 1;
            break;
        }
    }
    return 0;
}

void Robot::update_collision_com_timer(int time)
{
    _collision_com_timer = time;
}

void Robot::main_executor()
{

    int proceed = 0;
    switch (_STATE)
    {
    case 0:
        check_task();
        if (_STATE != 4)
            action_decoder();
        break;
    case 1:
        proceed = _loc.forward(_dist, _error);
        //      Serial.println("Moving forward");
        break;
    case 2:
        proceed = _loc.turn(_turn, 'L');
        //      Serial.println("turning left");
        break;
    case 3:
        proceed = _loc.turn(_turn, 'R');
        //      Serial.println("turning right");
        break;
    case 4:
        //      Serial.println("arrived");

        break;
    }
    if (proceed)
    {
        if (_STATE == 2 || _STATE == 3)
        {
            // wait for 1000ms for the next update
            _prev_time = millis();
            _turn_update = 0;
        }
        _STATE = 0;
    }
}

/*
  same as the previous action decoder
  take in the upcoming destination in _route
  output the necessary action to get to the destination
*/

void Robot::action_decoder()
{
    int *pos = get_pos();
    int x = _route[_ptr][0] - pos[0];
    int y = _route[_ptr][1] - pos[1];
    int ORI = pos[2];
    free(pos);
    Serial.print("Current orientation: ");
    Serial.println(ORI);
    _turn = 90;
    if (!_turn_update)
    {
        // if robot has just exit the turning state
        // hold for the synchronization from the camera
        _STATE = 0;
        _dist = 0;
        _turn = 0;
    }
    else if (abs(ORI) <= 2)
    {
        if (y >= 200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (y <= -1)
        {
            _STATE = 1;
            _direction = 'N';
            _dist = abs(y);
        }
        else
        {
            if (x >= 1)
            {
                _STATE = 3;
            }
            else if (x <= -1)
            {
                _STATE = 2;
            }
        }
    }
    else if (abs(ORI) >= 178)
    {
        if (y >= 1)
        {
            _STATE = 1;
            _direction = 'S';
            _dist = abs(y);
        }
        else if (y <= -200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else
        {
            if (x >= 1)
            {
                _STATE = 2;
            }
            else if (x <= -1)
            {
                _STATE = 3;
            }
        }
    }
    else if (ORI >= 88 && ORI <= 92)
    {
        if (x >= 1)
        {
            _STATE = 1;
            _direction = 'E';
            _dist = abs(x);
        }
        else if (x <= -200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else
        {
            if (y >= 1)
            {
                _STATE = 3;
            }
            else if (y <= -1)
            {
                _STATE = 2;
            }
        }
    }
    else if (ORI >= -92 && ORI <= -88)
    {
        if (x >= 200)
        {
            _STATE = 3;
            _turn = 180;
        }
        else if (x <= -1)
        {
            _STATE = 1;
            _direction = 'W';
            _dist = abs(x);
        }
        else
        {
            if (y >= 1)
            {
                _STATE = 2;
            }
            else if (y <= -1)
            {
                _STATE = 3;
            }
        }
    }
    else
    {
        int remain = ORI % 90;
        if (abs(remain) >= 45)
        {
            _STATE = (remain < 0) ? 2 : 3;
            _turn = 90 - abs(remain);
        }
        else
        {
            _STATE = (remain > 0) ? 2 : 3;
            _turn = abs(remain);
        }
    }
}

void Robot::check_task()
{
    int *pos = get_pos();
    if (abs(pos[0] - _route[_ptr][0]) <= 300 && abs(pos[1] - _route[_ptr][1]) <= 300)
    {
        Serial.println("arrived at a dst");
        Serial.print("Current pointer: ");
        Serial.println(_ptr);
        Serial.print("Total pointer:   ");
        Serial.println(_task_size);
        Serial.println();
        if (_ptr + 1 == _task_size)
        {
            //  robot arriving at the final destination
            _STATE = 4;
            Serial.println("Proceeding to free the memory");
            for (int i = 0; i < _task_size; i++)
                free(_route[i]);
            free(_route);
            _task_size = 0;
            _ptr = 1;
            _turn = 0;
            _dist = 0;
            _error = 0;
            Serial.println("All assets freed");
        }
        else
        {
            _ptr++;
        }
    }
    free(pos);
}

void Robot::collision_com_init(char *jsonStr)
{
    update_collision_com_state(1);
    // char *state;
    // return state;
    _communication.collision_com_init(jsonStr, _route[_ptr], get_pos(), get_id());
}

void Robot::collision_com_reply(char *jsonStr, char id, int action)
{
    // here determine the action to take in face of the collision
    _communication.collision_com_reply(jsonStr, get_pos(), action, id);
}

char *ssid = "nowifi";
char *password = "durf2020";

Robot robot(&encoder1, &encoder2, ssid, password);
AsyncUDP udp;

int ctr = 0;

void setup()
{
    //  SET UP THE ID OF THE ROBOT HERE
    robot.robot_init();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.begin(115200);
    // for testing purposes
    robot.update_state(0);
    int pos_n[3] = {3000, 0, 90};
    robot.update_abs(pos_n);
    robot.auto_route();

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("WiFi Failed");
        while (1)
        {
            delay(1000);
        }
    }
    if (udp.listenMulticast(IPAddress(224, 3, 29, 1), 10001))
    {
        udp.onPacket([](AsyncUDPPacket packet) {
            DynamicJsonDocument jInfo(1024);
            deserializeJson(jInfo, packet.data());
            const int Purpose = jInfo["Purpose"];
            switch (Purpose)
            {
            case 1:
            {
                const char *ID;
                ID = "7";
                if (!jInfo[ID][0][0] && !jInfo[ID][0][1])
                    break;
                int pos_n[3];
                for (int i = 0; i < 2; i++)
                    pos_n[i] = jInfo[ID][0][i];
                pos_n[2] = jInfo[ID][1];
                robot.update_abs(pos_n);
                if (robot.get_state() == 1)
                    robot.calc_error();
                char jsonStr[80];
                //              jsonCreator(jsonStr);
                if (!ctr)
                {
                    robot.update_state(0);
                    encoder1.numberTicks = 0;
                    encoder2.numberTicks = 0;
                    Serial.println("routing the robot");
                    robot.auto_route();
                    ctr++;
                }
                break;
            }
            case 4:
            {
                robot.update_state(1);
                if (robot.get_state() == 1)
                {
                    int pos[2], route[2];
                    char ID = jInfo["ID"];
                    for (int i = 0; i < 2; i++)
                    {
                        pos[i] = jInfo["pos"][i];
                        route[i] = jInfo["route"][i];
                    }
                    int *coordinate = robot.process_route(pos, route);
                    Serial.print("Collision coordinates: ");
                    Serial.print(coordinate[0]);
                    Serial.print(" ");
                    Serial.println(coordinate[1]);
                    // collision avoidance only if the coordinate is between the current position and the next destination of the robots
                    if (coordinate[0])
                    {
                        // there is already a communication between the two robots
                        // robot.update_collision_com_state(2);
                        // there is a possible collision
                        // com state makes sure no ir readings would be taken into account
                        int value;
                        char direction;
                        robot.trajectory_calculator(&direction, &value, pos, route);
                        int *action = robot.collision_action(pos, coordinate, &direction);
                        char msg[80];

                        // DynamicJsonDocument doc(1024);
                        // doc["Purpose"] = 5;
                        // doc["action"] = action[1];
                        // doc["ID"] = 7;
                        // serializeJson(doc, jsonStr);
                        // doc.clear();
                        robot.collision_com_reply(msg, ID, action[1]);
                        udp.writeTo((const uint8_t *)msg, strlen(msg), IPAddress(224, 3, 29, 1), 10001);
                        free(coordinate);
                        free(action);
                    }
                }
                robot.update_state(0);
            }
            case 5:
            {
                if (jInfo["ID"] == robot.get_id())
                {
                    int action = jInfo["action"];
                }
            }
            }
        });
    }
}

void loop()
{
    robot.check_collision();
    if (robot.get_collision_state() == 3 && !robot.get_collision_com_state() && millis() - robot.get_collision_com_start_time() > robot.get_collision_com_timer())
    {
        char jsonStr[80];
        robot.collision_com_init(jsonStr);
        udp.writeTo((const uint8_t *)jsonStr, strlen(jsonStr), IPAddress(224, 3, 29, 1), 10001);
    }
    // if (robot.get_state() != -1)
    //     robot.main_executor();
}
