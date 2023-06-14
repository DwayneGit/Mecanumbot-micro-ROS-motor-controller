#pragma once

#ifndef MOTOR_H_
#define MOTOR_H_

#include "pico/stdlib.h"
#include "pwm.h"
#include "common.h"
#include "motor_state.h"

struct Driver {
    struct pin_pair motor_pins;
    uint16_t pwm_period;
};

struct Driver * driver_init(const struct pin_pair * pins);
void driver_destroy(struct Driver * this);

void dual_PWM_driver_init(struct Driver *this, pwm_config *pwm_cfg, uint16_t period);
void dual_PWM_driver_update_frequency(struct Driver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode);
void dual_PWM_driver_apply_duty(struct Driver *this, float duty, enum DecayMode mode);

struct Motor {
    struct pin_pair motor_pins;
    uint16_t pwm_period;

    struct Driver *driver;
    struct MotorState state;
    pwm_config pwm_cfg;
    float pwm_frequency;
    enum DecayMode motor_mode;
};

struct Motor * motor_init(
    const struct pin_pair * pins,
    enum Direction direction, 
    float speed_scale, 
    float zeropoint,
    float deadzone, 
    float freq, 
    enum DecayMode mode
);

struct Motor * motor_init_default(
    const struct pin_pair * pins
);

void motor_destroy(struct Motor* this);

bool init(struct Motor* this);

// For print access in micropython
struct pin_pair pins(struct Motor* this);

void motor_enable(struct Motor* this);
void motor_disable(struct Motor* this);
bool motor_is_enabled(struct Motor* this);

float get_motor_duty(struct Motor* this);
void set_motor_duty(struct Motor* this, float duty);

float get_motor_speed(struct Motor* this);
void set_motor_speed(struct Motor* this, float speed);

bool set_motor_frequency(struct Motor* this, float freq);

//--------------------------------------------------

void motor_stop(struct Motor* this);
void motor_coast(struct Motor* this);
void motor_brake(struct Motor* this);
void motor_full_negative(struct Motor* this);
void motor_full_positive(struct Motor* this);

void motor_to_percent(struct Motor* this, float in, float in_min, float in_max);
void motor_to_percent_default(struct Motor* this, float in);
void motor_to_percent_w_speed(struct Motor* this, float in, float in_min, float in_max, float speed_min, float speed_max);

//--------------------------------------------------

enum Direction get_motor_direction(struct Motor* this);
void set_motor_direction(struct Motor* this, enum Direction direction);

float get_motor_speed_scale(struct Motor* this);
void set_motor_speed_scale(struct Motor* this, float speed_scale);

float get_motor_zeropoint(struct Motor* this);
void set_motor_zeropoint(struct Motor* this, float zeropoint);

float get_motor_deadzone(struct Motor* this);
void set_motor_deadzone(struct Motor* this, float deadzone);

enum DecayMode get_motor_decay_mode(struct Motor* this);
void set_motor_decay_mode(struct Motor* this, enum DecayMode mode);

#endif