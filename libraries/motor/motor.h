#ifndef MOTOR_H_
#define MOTOR_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "common.h"
#include "motor_state.h"

struct CDriver {
    const pin_pair * motor_pins;
    uint16_t pwm_period;

    void (*init)(const struct CDriver *this, pwm_config *pwm_cfg, uint16_t period);
    void (*update_frequency)(const struct CDriver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode);
    void (*apply_duty)(const struct CDriver *this, float duty, enum DecayMode mode);
} ;
    // void (*driver_init)(Driver *this, pwm_config *pwm_cfg, uint16_t period);
    // void (*driver_update_frequency)(Driver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode);
    // void (*driver_apply_duty)(Driver *this, float duty, enum DecayMode mode);

void driver_new(struct CDriver *this, const pin_pair * pins);
void driver_destroy(struct CDriver *this);
void driver_init(const struct CDriver *this, pwm_config *pwm_cfg, uint16_t period);
void driver_update_frequency(const struct CDriver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode);
void driver_apply_duty(const struct CDriver *this, float duty, enum DecayMode mode);

typedef struct {
    struct CDriver driver;
} CDualPWMDriver;

void dualpwmdriver_new(CDualPWMDriver *this, const pin_pair * pins);
void dualpwmdriver_destroy(CDualPWMDriver *this);
void dualpwmdriver_init(CDualPWMDriver *this, pwm_config *pwm_cfg, uint16_t period);
void dualpwmdriver_update_frequency(CDualPWMDriver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode);
void dualpwmdriver_apply_duty(CDualPWMDriver *this, float duty, enum DecayMode mode);

// typedef struct {
//     struct CDriver driver;
// } PhEnDriver;

// struct CDriver * cdriver_init(const pin_pair * pins);
// void driver_destroy(struct CDriver * this);

typedef struct{
    const pin_pair * motor_pins;
    uint16_t pwm_period;

    struct CDriver *driver;
    struct MotorState state;
    pwm_config pwm_cfg;
    float pwm_frequency;
    enum DecayMode motor_mode;
} Motor;

void motor_new(
    Motor * motor,
    const pin_pair * pins,
    enum Direction direction, 
    float speed_scale, 
    float zeropoint,
    float deadzone, 
    float freq, 
    enum DecayMode mode,
    bool ph_en_driver
);

void motor_new_default(
    Motor * motor,
    const pin_pair * pins
);

void motor_destroy(Motor* this);

bool motor_init(Motor* this);

// For print access in micropython
const pin_pair * pins(Motor* this);

void motor_enable(Motor* this);
void motor_disable(Motor* this);
bool motor_is_enabled(Motor* this);

float get_motor_duty(Motor* this);
void set_motor_duty(Motor* this, float duty);

float get_motor_speed(Motor* this);
void set_motor_speed(Motor* this, float speed);

bool set_motor_frequency(Motor* this, float freq);

//--------------------------------------------------

void motor_stop(Motor* this);
void motor_coast(Motor* this);
void motor_brake(Motor* this);
void motor_full_negative(Motor* this);
void motor_full_positive(Motor* this);

void motor_to_percent(Motor* this, float in, float in_min, float in_max);
void motor_to_percent_default(Motor* this, float in);
void motor_to_percent_w_speed(Motor* this, float in, float in_min, float in_max, float speed_min, float speed_max);

//--------------------------------------------------

enum Direction get_motor_direction(Motor* this);
void set_motor_direction(Motor* this, enum Direction direction);

float get_motor_speed_scale(Motor* this);
void set_motor_speed_scale(Motor* this, float speed_scale);

float get_motor_zeropoint(Motor* this);
void set_motor_zeropoint(Motor* this, float zeropoint);

float get_motor_deadzone(Motor* this);
void set_motor_deadzone(Motor* this, float deadzone);

enum DecayMode get_motor_decay_mode(Motor* this);
void set_motor_decay_mode(Motor* this, enum DecayMode mode);

#endif