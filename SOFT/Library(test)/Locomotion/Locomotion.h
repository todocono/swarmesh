/*
Locomotion.h - Library for Swarmesh

Before constructing class, create two instances of Encoders and pass 
their ADDRESSES into Locomotion's constructor

Need to figure out a way to pass the interrupt function into the 
motor_init() function
*/


#ifndef Locomotion_h
#define Locomotion_h

#include "Arduino.h"

struct Encoder;

void IRAM_ATTR isr1();
void IRAM_ATTR isr2();

class Motor
{
private:
    Encoder *_encoder;
    int _PWM;
    int _DIR;
    int _IDX;
    uint8_t _PIN;
    void _reset_encoder();

public:
    Motor(Encoder *encoder, int PWM, int DIR, int IDX);
    void motor_move(int spd, int dir);
    void motor_stop();
    const int get_idx();
    const int get_pwm();
    const int get_dir();
    const int get_tick();
    const uint8_t get_pin();
};

class Locomotion
{
private:
    Motor _motor1;
    Motor _motor2;
    Motor *_motors;
    int _PULSE;
    void _forward(int dist);
    void _turn(int deg, char dir);
public:
    Locomotion(Encoder *encoder1, Encoder *encoder2);
    void motor_init();
};

#endif