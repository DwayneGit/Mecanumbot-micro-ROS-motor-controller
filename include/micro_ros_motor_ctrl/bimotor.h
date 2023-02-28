#ifndef BIMOTOR_HPP
#define BIMOTOR_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define BIMOTOR_FORWARD false
#define BIMOTOR_BACKWARD true

#define ENCODER_MIN -32768
#define ENCODER_MAX 32767

struct BiMotor
{
    uint gpioForward;
    uint gpioBackward;
    uint gpioEnabled;
    uint gpioEncOut;
    uint slice;
    uint ENchan;
    uint direction;
    uint speed;
    uint freq;
    uint resolution;
    int16_t encoderTickCount;
    int motorNum;
    bool on;
};

void set_motor_off(struct BiMotor *this);
void bimotor_destroy(struct BiMotor *this);
void get_encoder_data(struct BiMotor *this);
void set_motor_direction(struct BiMotor *this, bool direction);
void set_motor_speed(struct BiMotor *this, double vel, bool direction);
struct BiMotor * bimotor_init(
    int motNum, 
    uint gpioEn, 
    uint gpioFor, 
    uint gpioBack, 
    uint gpioEncoderOut,
    uint frequency
);

#endif