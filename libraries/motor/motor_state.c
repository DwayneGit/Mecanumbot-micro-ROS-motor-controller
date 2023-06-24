#include "motor_state.h"
#include "common.h"
#include "float.h"
#include "math.h"

struct MotorState * motor_state_const(
  enum Direction direction, 
  float speed_scale, 
  float zeropoint, 
  float deadzone
){
  struct MotorState * ms;
  ms = (struct MotorState *) malloc(sizeof(struct MotorState));
  
  ms->motor_speed = (0.0f); 
  ms->last_enabled_duty = (0.0f); 
  ms->enabled = (false);
  ms->motor_direction = (direction);
  ms->motor_scale = (speed_scale);
  ms->motor_zeropoint = (zeropoint);
  ms->motor_deadzone = (deadzone);
  
  return ms;
}

struct MotorState * motor_state_init_default(){
  return motor_state_const(
    NORMAL_DIR,
    DEFAULT_SPEED_SCALE,
    DEFAULT_ZEROPOINT,
    DEFAULT_DEADZONE
  );
}

struct MotorState * motor_state_init(enum Direction direction, float speed_scale, float zeropoint, float deadzone){
  return motor_state_const(
    direction,
    MAX(speed_scale, FLT_EPSILON),
    CLAMP(zeropoint, 0.0f, 1.0f - FLT_EPSILON),
    CLAMP(deadzone, 0.0f, 1.0f)
  );
}

float enable_with_return(struct MotorState * this) {
  this->enabled = true;
  return get_deadzoned_duty(this);
}

float disable_with_return(struct MotorState * this) {
   this->enabled = false;
   return NAN;
}

bool is_enabled(struct MotorState * this) {
  return this->enabled;
}

float get_duty(struct MotorState * this) {
  return (this->motor_direction == NORMAL_DIR) ? this->last_enabled_duty : 0.0f - this->last_enabled_duty;
}

float get_deadzoned_duty(struct MotorState * this) {
  float duty = 0.0f;
  if((this->last_enabled_duty <= 0.0f - this->motor_deadzone) || (this->last_enabled_duty >= this->motor_deadzone)) {
    duty = this->last_enabled_duty;
  }
  if(this->enabled)
    return duty;
  else
    return NAN;
}

float set_duty_with_return(struct MotorState * this, float duty) {
  // Invert provided speed if the motor direction is reversed
  if(this->motor_direction == REVERSED_DIR)
    duty = 0.0f - duty;
  // Clamp the duty between the hard limits
  this->last_enabled_duty = CLAMP(duty, -1.0f, 1.0f);
  // Calculate the corresponding speed
  this->motor_speed = duty_to_speed(this->last_enabled_duty, this->motor_zeropoint, this->motor_scale);
  return enable_with_return(this);
}

float get_speed(struct MotorState * this) {
  return (this->motor_direction == NORMAL_DIR) ? this->motor_speed : 0.0f - this->motor_speed;
}

float set_speed_with_return(struct MotorState * this, float speed) {
  // Invert provided speed if the motor direction is reversed
  if(this->motor_direction == REVERSED_DIR)
    speed = 0.0f - speed;
  // Clamp the speed between the hard limits
  this->motor_speed = CLAMP(speed, 0.0f - this->motor_scale, this->motor_scale);
  // Calculate the corresponding duty cycle
  this->last_enabled_duty = speed_to_duty(this->motor_speed, this->motor_zeropoint, this->motor_scale);
  return enable_with_return(this);
}

float stop_with_return(struct MotorState * this) {
  return set_duty_with_return(this, 0.0f);
}

float full_negative_with_return(struct MotorState * this) {
  return set_duty_with_return(this, -1.0f);
}

float full_positive_with_return(struct MotorState * this) {
  return set_duty_with_return(this, 1.0f);
}

float to_percent_with_return(struct MotorState * this, float in, float in_min, float in_max) {
  float speed = map_float(in, in_min, in_max, 0.0f - this->motor_scale, this->motor_scale);
  return set_speed_with_return(this, speed);
}

float to_percent_with_return_default(struct MotorState * this, float in){
  return to_percent_with_return(this, in, ZERO_PERCENT, ONEHUNDRED_PERCENT);
}

float to_percent_with_return_w_speed(struct MotorState * this, float in, float in_min, float in_max, float speed_min, float speed_max) {
  float speed = map_float(in, in_min, in_max, speed_min, speed_max);
  return set_speed_with_return(this, speed);
}

enum Direction get_direction(struct MotorState * this) {
  return this->motor_direction;
}

void set_direction(struct MotorState * this, enum Direction direction) {
  this->motor_direction = direction;
}

float get_speed_scale(struct MotorState * this) {
  return this->motor_scale;
}

void set_speed_scale(struct MotorState * this, float speed_scale) {
  this->motor_scale = MAX(speed_scale, FLT_EPSILON);
  this->motor_speed = duty_to_speed(this->last_enabled_duty, this->motor_zeropoint, this->motor_scale);
}

float get_zeropoint(struct MotorState * this) {
  return this->motor_scale;
}

void set_zeropoint(struct MotorState * this, float zeropoint) {
  this->motor_zeropoint = CLAMP(zeropoint, 0.0f, 1.0f - FLT_EPSILON);
  this->motor_speed = duty_to_speed(this->last_enabled_duty, this->motor_zeropoint, this->motor_scale);
}

float get_deadzone(struct MotorState * this) {
  return this->motor_deadzone;
}

float set_deadzone_with_return(struct MotorState * this, float deadzone) {
  this->motor_deadzone = CLAMP(deadzone, 0.0f, 1.0f);
  return get_deadzoned_duty(this);
}

//--------------------------------------------------

int32_t duty_to_level(float duty, uint32_t resolution) {
  return (int32_t)(duty * (float)resolution);
}

float map_float(float in, float in_min, float in_max, float out_min, float out_max) {
  return (((in - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}

float duty_to_speed( float duty, float zeropoint, float scale) {
  float speed = 0.0f;
  if(duty > zeropoint) {
    speed = map_float(duty, zeropoint, 1.0f, 0.0f, scale);
  }
  else if(duty < -zeropoint) {
    speed = map_float(duty, -zeropoint, -1.0f, 0.0f, -scale);
  }
  return speed;
}

float speed_to_duty(float speed, float zeropoint, float scale) {
  float duty = 0.0f;
  if(speed > 0.0f) {
    duty = map_float(speed, 0.0f, scale, zeropoint, 1.0f);
  }
  else if(speed < 0.0f) {
    duty = map_float(speed, 0.0f, -scale, -zeropoint, -1.0f);
  }
  return duty;
}
