#include "motor.h"
#include "hardware/clocks.h"
#include "pwm.h"
#include "math.h"
#include "motor_state.h"

struct Driver * driver_init(const struct pin_pair * pins) {
    struct Driver * driver;
    driver = (struct Driver *) malloc(sizeof(struct Driver));
    
    driver->motor_pins = *pins;
    driver->pwm_period = 1;
    
    return driver;
}

void driver_destroy(struct Driver * this) {
    gpio_set_function(this->motor_pins.first, GPIO_FUNC_NULL);
    gpio_set_function(this->motor_pins.second, GPIO_FUNC_NULL);
}

void dual_PWM_driver_init(struct Driver *this, pwm_config *pwm_cfg, uint16_t period) {
    this->pwm_period = period;

    // Set up the positive and negative pins
    pwm_init(pwm_gpio_to_slice_num(this->motor_pins.positive), pwm_cfg, true);
    pwm_init(pwm_gpio_to_slice_num(this->motor_pins.negative), pwm_cfg, true);

    gpio_set_function(this->motor_pins.positive, GPIO_FUNC_PWM);
    gpio_set_function(this->motor_pins.negative, GPIO_FUNC_PWM);

    pwm_set_gpio_level(this->motor_pins.positive, 0);
    pwm_set_gpio_level(this->motor_pins.negative, 0);
}

void dual_PWM_driver_update_frequency(struct Driver *this, uint8_t div, uint8_t mod, uint16_t period, float duty, enum DecayMode mode) {
    // Record if the new period will be larger or smaller.
    // This is used to apply new pwm speeds either before or after the wrap is applied,
    // to avoid momentary blips in PWM output on SLOW_DECAY
    bool pre_update_pwm = (period > this->pwm_period);
    this->pwm_period = period;

    uint pos_pin_slice = pwm_gpio_to_slice_num(this->motor_pins.positive);
    uint neg_pin_slice = pwm_gpio_to_slice_num(this->motor_pins.negative);

    // Apply the new divider
    pwm_set_clkdiv_int_frac(pos_pin_slice, div, mod);
    if((neg_pin_slice != pos_pin_slice))
        pwm_set_clkdiv_int_frac(neg_pin_slice, div, mod);

    // If the period is larger, update the pwm before setting the new wraps
    if(pre_update_pwm) {
        dual_PWM_driver_apply_duty(this, duty, mode);
    }

    // Set the new wrap (should be 1 less than the period to get full 0 to 100%)
    pwm_set_wrap(pos_pin_slice, this->pwm_period - 1);
    if(neg_pin_slice != pos_pin_slice)
        pwm_set_wrap(neg_pin_slice, this->pwm_period - 1);

    // If the period is smaller, update the pwm after setting the new wraps
    if(!pre_update_pwm) {
        dual_PWM_driver_apply_duty(this, duty, mode);
    }
}

void dual_PWM_driver_apply_duty(struct Driver *this, float duty, enum DecayMode mode) {
    if(isfinite(duty)) {
        int32_t signed_level = duty_to_level(duty, this->pwm_period);

        switch(mode) {
        case SLOW_DECAY: //aka 'Braking'
            if(signed_level >= 0) {
                pwm_set_gpio_level(this->motor_pins.positive, this->pwm_period);
                pwm_set_gpio_level(this->motor_pins.negative, this->pwm_period - signed_level);
            }
            else {
                pwm_set_gpio_level(this->motor_pins.positive, this->pwm_period + signed_level);
                pwm_set_gpio_level(this->motor_pins.negative, this->pwm_period);
            }
        break;

        case FAST_DECAY: //aka 'Coasting'
        default:
            if(signed_level >= 0) {
                pwm_set_gpio_level(this->motor_pins.positive, signed_level);
                pwm_set_gpio_level(this->motor_pins.negative, 0);
            }
            else {
                pwm_set_gpio_level(this->motor_pins.positive, 0);
                pwm_set_gpio_level(this->motor_pins.negative, 0 - signed_level);
            }
        break;
        }
    }
    else {
        pwm_set_gpio_level(this->motor_pins.positive, 0);
        pwm_set_gpio_level(this->motor_pins.negative, 0);
    }
}

struct Motor * motor_init(
    const struct pin_pair * pins, 
    enum Direction direction, 
    float speed_scale, 
    float zeropoint,
    float deadzone, 
    float freq, 
    enum DecayMode mode
){
    struct Motor * motor;
    motor = (struct Motor *) malloc(sizeof(struct Motor));
    
    motor->driver = driver_init(pins);
    motor->state = * motor_state_init(direction, speed_scale, zeropoint, deadzone);
    motor->pwm_frequency = freq;
    motor->motor_mode = mode;
    
    return motor;
}

struct Motor * motor_init_default(
    const struct pin_pair * pins
){
    return motor_init(
        pins, 
        NORMAL_DIR, 
        DEFAULT_SPEED_SCALE, 
        DEFAULT_ZEROPOINT,
        DEFAULT_DEADZONE, 
        DEFAULT_FREQUENCY, 
        DEFAULT_DECAY_MODE
    );
}

void motor_destroy(struct Motor* this){
    driver_destroy(this->driver);
}

bool init(struct Motor* this) {
    bool success = false;

    uint16_t period; 
    uint16_t div16;
    if(calculate_pwm_factors(this->pwm_frequency, &period, &div16)) {
      this->pwm_cfg = pwm_get_default_config();

      // Set the new wrap (should be 1 less than the period to get full 0 to 100%)
      pwm_config_set_wrap(&this->pwm_cfg, period - 1);

      // Apply the divider
      pwm_config_set_clkdiv(&this->pwm_cfg, (float)div16 / 16.0f); // There's no 'pwm_config_set_clkdiv_int_frac' for some reason...

      // Set up the pin driver
      dual_PWM_driver_init(this->driver, &this->pwm_cfg, period);

      success = true;
    }

    return success;
}

struct pin_pair pins(struct Motor* this){
    return this->driver->motor_pins;
}

void motor_enable(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, enable_with_return(&this->state), this->motor_mode);
}

void motor_disable(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, disable_with_return(&this->state), this->motor_mode);
}

bool motor_is_enabled(struct Motor* this){
    return is_enabled(&this->state);
}

float get_motor_duty(struct Motor* this){
    return get_duty(&this->state);
}

void set_motor_duty(struct Motor* this, float duty) {
    dual_PWM_driver_apply_duty(this->driver, set_duty_with_return(&this->state, duty), this->motor_mode);
}

float get_motor_speed(struct Motor* this){
    return get_speed(&this->state);
}

void set_motor_speed(struct Motor* this, float speed) {
    dual_PWM_driver_apply_duty(this->driver, set_speed_with_return(&this->state, speed), this->motor_mode);
}

bool get_motor_frequency(struct Motor* this, float freq) {
    bool success = false;

    if((freq >= MIN_FREQUENCY) && (freq <= MAX_FREQUENCY)) {
      // Calculate a suitable pwm wrap period for this frequency
      uint16_t period; uint16_t div16;
      if(calculate_pwm_factors(freq, &period, &div16)) {

        this->pwm_frequency = freq;

        uint8_t div = div16 >> 4;
        uint8_t mod = div16 % 16;
        dual_PWM_driver_update_frequency(this->driver,div, mod, period, get_deadzoned_duty(&this->state), this->motor_mode);

        success = true;
      }
    }
    return success;
}

void motor_stop(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, stop_with_return(&this->state), this->motor_mode);
}

void motor_coast(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, stop_with_return(&this->state), FAST_DECAY);
}

void motor_brake(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, stop_with_return(&this->state), SLOW_DECAY);
}

void motor_full_negative(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, full_negative_with_return(&this->state), this->motor_mode);
}

void motor_full_positive(struct Motor* this) {
    dual_PWM_driver_apply_duty(this->driver, full_positive_with_return(&this->state), this->motor_mode);
}

void motor_to_percent(struct Motor* this, float in, float in_min, float in_max) {
    dual_PWM_driver_apply_duty(this->driver, to_percent_with_return(&this->state, in, in_min, in_max), this->motor_mode);
}

void motor_to_percent_default(struct Motor* this, float in) {
    dual_PWM_driver_apply_duty(this->driver, to_percent_with_return(&this->state, in, ZERO_PERCENT, ONEHUNDRED_PERCENT), this->motor_mode);
}

void motor_to_percent_w_speed(struct Motor* this, float in, float in_min, float in_max, float speed_min, float speed_max) {
    dual_PWM_driver_apply_duty(this->driver, to_percent_with_return_w_speed(&this->state, in, in_min, in_max, speed_min, speed_max), this->motor_mode);
}

enum Direction get_motor_direction(struct Motor* this){
    return get_direction(&this->state);
}

void set_motor_direction(struct Motor* this, enum Direction direction) {
    set_direction(&this->state, direction);
}

float get_motor_speed_scale(struct Motor* this){
    return get_speed_scale(&this->state);
}

void set_motor_speed_scale(struct Motor* this, float speed_scale) {
    set_speed_scale(&this->state, speed_scale);
}

float get_motor_zeropoint(struct Motor* this){
    return get_zeropoint(&this->state);
}

void set_motor_zeropoint(struct Motor* this, float zeropoint) {
    set_zeropoint(&this->state, zeropoint);
}

float get_motor_deadzone(struct Motor* this){
    return get_deadzone(&this->state);
}

void set_motor_deadzone(struct Motor* this, float deadzone) {
    dual_PWM_driver_apply_duty(this->driver, set_deadzone_with_return(&this->state, deadzone), this->motor_mode);
}

enum DecayMode get_motor_decay_mode(struct Motor* this) {
    return this->motor_mode;
}

void set_motor_decay_mode(struct Motor* this, enum DecayMode mode) {
    this->motor_mode = mode;
    dual_PWM_driver_apply_duty(this->driver, get_deadzoned_duty(&this->state), this->motor_mode);
}
