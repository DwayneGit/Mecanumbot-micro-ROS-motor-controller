#include "pico/stdlib.h"
#include <math.h>

#include "motor2040.h"

/*
Demonstrates how to create a Motor object and control it.
*/

// How many sweeps of the motor to perform
const uint SWEEPS = 2;

// The number of discrete sweep steps
const uint STEPS = 10;

// The time in milliseconds between each step of the sequence
const uint STEPS_INTERVAL_MS = 500;

// How far from zero to drive the motor when sweeping
const float SPEED_EXTENT = 1.0f;

// const struct pin_pair * MOTOR_A
struct Motor * m;

int main() {
// Create a motor
  MOTOR_A = pin_pair_init(MOTOR_A_P, MOTOR_A_N);
  m = motor_init_default(MOTOR_A);

  stdio_init_all();

  // Initialise the motor
  init(m);

  // Enable the motor
  motor_enable(m);
  sleep_ms(2000);

  // Drive at full positive
  motor_full_positive(m);
  sleep_ms(2000);

  // Stop moving
  motor_stop(m);
  sleep_ms(2000);

  // Drive at full negative
  motor_full_negative(m);
  sleep_ms(2000);

  // Coast to a gradual stop
  motor_coast(m);
  sleep_ms(2000);


  // Do a sine speed sweep
  for(uint j = 0u; j < SWEEPS; j++) {
    for(uint i = 0u; i < 360; i++) {
      set_motor_speed(m, sin(((float)i * (float)M_PI) / 180.0f) * SPEED_EXTENT);
      sleep_ms(20);
    }
  }

  // Do a stepped speed sweep
  for(uint j = 0u; j < SWEEPS; j++) {
    for(uint i = 0u; i < STEPS; i++) {
      motor_to_percent_w_speed(m, i, 0, STEPS, 0.0 - SPEED_EXTENT, SPEED_EXTENT);
      sleep_ms(STEPS_INTERVAL_MS);
    }
    for(uint i = 0u; i < STEPS; i++) {
      motor_to_percent_w_speed(m, i, STEPS, 0, 0.0 - SPEED_EXTENT, SPEED_EXTENT);
      sleep_ms(STEPS_INTERVAL_MS);
    }
  }

  // Disable the motor
  motor_disable(m);
}
