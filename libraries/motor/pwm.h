#pragma once

#ifndef PWM_H_
#define PWM_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"

bool calculate_pwm_factors(float freq, uint16_t* top_out, uint16_t* div16_out);

#endif