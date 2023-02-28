#include "../include/micro_ros_motor_ctrl/bimotor.h"
#include "../include/micro_ros_motor_ctrl/pwm.h"
#include "../include/micro_ros_motor_ctrl/events.h"
#include <stdlib.h>

struct BiMotor * bimotor_init(int motNum, uint gpioEn, uint gpioFor, uint gpioBack, uint gpioEncoderOut, uint frequency)
{
    gpio_set_function(gpioEn, GPIO_FUNC_PWM);

    gpio_init(gpioEncoderOut);
    gpio_set_dir(gpioEncoderOut, GPIO_IN);
    gpio_pull_down(gpioEncoderOut);

    gpio_init(gpioFor);
    gpio_set_dir(gpioFor, GPIO_OUT);

    gpio_init(gpioBack);
    gpio_set_dir(gpioBack, GPIO_OUT);

    uint slice = pwm_gpio_to_slice_num(gpioEn);
    uint ENchannel = pwm_gpio_to_channel(gpioEn);

    pwm_set_duty(slice, ENchannel, 0);

    struct BiMotor * biMotor;
    biMotor = (struct BiMotor *) malloc(sizeof(struct BiMotor));
    
    biMotor->gpioEnabled = gpioEn;
    biMotor->slice = slice;
    biMotor->ENchan = ENchannel;
    biMotor->gpioEncOut = gpioEncoderOut;
    biMotor->gpioForward = gpioFor;
    biMotor->gpioBackward = gpioBack;
    biMotor->motorNum = motNum;
    biMotor->freq = frequency;
    biMotor->speed = 0;
    biMotor->direction = BIMOTOR_FORWARD;
    biMotor->resolution = pwm_set_freq_duty(slice, ENchannel, frequency, 0);
    biMotor->encoderTickCount = 0;
    biMotor->on = true;
    
    pwm_set_enabled(biMotor->slice, biMotor->on);
    
    return biMotor;
}

void bimotor_destroy(struct BiMotor *this){
    free(this);
}

void set_motor_speed(struct BiMotor *this, double vel, bool direction)
{
    set_motor_direction(this, direction);
    pwm_set_duty(this->slice, this->ENchan, vel);
    this->speed = vel;
}

void set_motor_off(struct BiMotor *this)
{
    set_motor_speed(this, 0, BIMOTOR_FORWARD);
    this->on = false;
    pwm_set_enabled(this->slice, this->on);
}

void set_motor_direction(struct BiMotor *this, bool direction)
{
    if (direction == BIMOTOR_FORWARD)
    {
        gpio_put(this->gpioBackward, 0);
        gpio_put(this->gpioForward, 1);
    }
    else
    {
        gpio_put(this->gpioBackward, 1);
        gpio_put(this->gpioForward, 0);
    }
    this->direction = direction;
}

void get_encoder_data(struct BiMotor *this){
    if(this->direction == BIMOTOR_BACKWARD){
        if(this->encoderTickCount == ENCODER_MIN)
            this->encoderTickCount == ENCODER_MAX;
        else this->encoderTickCount--;
    }
    else {
        if(this->encoderTickCount == ENCODER_MAX)
            this->encoderTickCount == ENCODER_MIN;
        else this->encoderTickCount++;
    }
    // printf("%i %i\n\r", this->motorNum, this->encoderTickCount);
}