#pragma once

#ifndef MOTOR_STATE_H_
#define MOTOR_STATE_H_

#include "pico/stdlib.h"
#include "common.h"

enum DecayMode {
    FAST_DECAY  = 0, //aka 'Coasting'
    SLOW_DECAY  = 1, //aka 'Braking'
};

const float DEFAULT_SPEED_SCALE = 1.0f;        // The standard motor speed scale
const float DEFAULT_ZEROPOINT = 0.0f;          // The standard motor zeropoint
const float DEFAULT_DEADZONE = 0.05f;          // The standard motor deadzone

const enum DecayMode DEFAULT_DECAY_MODE = SLOW_DECAY;   // The standard motor decay behaviour
const float DEFAULT_FREQUENCY = 25000.0f;      // The standard motor update rate
const float MIN_FREQUENCY = 10.0f;
const float MAX_FREQUENCY = 400000.0f;

const float ZERO_PERCENT = 0.0f;
const float ONEHUNDRED_PERCENT = 1.0f;

struct MotorState {
    float motor_speed;
    float last_enabled_duty;
    bool enabled;

    // Customisation variables
    enum Direction motor_direction;
    float motor_scale;
    float motor_zeropoint;
    float motor_deadzone;
};

struct MotorState * motor_state_init_default();
struct MotorState * motor_state_init(enum Direction direction, float speed_scale, float zeropoint, float deadzone);

void motor_state_destroy(enum Direction direction, float speed_scale, float zeropoint, float deadzone);

float enable_with_return(struct MotorState * this);
float disable_with_return(struct MotorState * this);
bool is_enabled(struct MotorState * this);

float get_duty(struct MotorState * this);
float get_deadzoned_duty(struct MotorState * this);
float set_duty_with_return(struct MotorState * this, float duty);

float get_speed(struct MotorState * this);
float set_speed_with_return(struct MotorState * this, float speed);

//--------------------------------------------------

float stop_with_return(struct MotorState * this);
float full_negative_with_return(struct MotorState * this);
float full_positive_with_return(struct MotorState * this);
float to_percent_with_return(struct MotorState * this, float in, float in_min, float in_max);
float to_percent_with_return_w_speed(struct MotorState * this, float in, float in_min, float in_max, float speed_min, float speed_max);
float to_percent_with_return_default(struct MotorState * this, float in);

//--------------------------------------------------

enum Direction get_direction(struct MotorState * this);
void set_direction(struct MotorState * this, enum Direction direction);

float get_speed_scale(struct MotorState * this);
void set_speed_scale(struct MotorState * this, float speed_scale);

float get_zeropoint(struct MotorState * this);
void set_zeropoint(struct MotorState * this, float zeropoint);

float get_deadzone(struct MotorState * this);
float set_deadzone_with_return(struct MotorState * this, float deadzone);

//--------------------------------------------------

int32_t duty_to_level(float duty, uint32_t resolution);

float map_float(float in, float in_min, float in_max, float out_min, float out_max);


float duty_to_speed(float duty, float zeropoint, float scale);
float speed_to_duty(float speed, float zeropoint, float scale);

#endif