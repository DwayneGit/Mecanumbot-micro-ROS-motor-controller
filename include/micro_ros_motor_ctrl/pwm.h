#ifndef PWM_HPP
#define PWM_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d);
uint32_t pwm_get_wrap(uint slice_num);
void pwm_set_duty(uint slice_num, uint chan, int d);

#endif