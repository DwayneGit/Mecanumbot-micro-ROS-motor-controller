#include "../include/micro_ros_motor_ctrl/mecanumbot.h"
#include <stdlib.h>
#include <math.h>

struct Mecanumbot * mecanumbot_init()
{
    struct Mecanumbot * mbot;
    mbot = (struct Mecanumbot *) malloc(sizeof(struct Mecanumbot));

    mbot->on = false;
    mbot->direction = MECANUMBOT_DIRECTION_FORWARD;

    mbot->motor1 = bimotor_init(1, BRIDGE1_ENB, BRIDGE1_IN3, BRIDGE1_IN4, BRIDGE1_ENCA, 2000);
    mbot->motor2 = bimotor_init(2, BRIDGE1_ENA, BRIDGE1_IN1, BRIDGE1_IN2, BRIDGE1_ENCB, 2000);
    mbot->motor3 = bimotor_init(3, BRIDGE2_ENA, BRIDGE2_IN1, BRIDGE2_IN2, BRIDGE2_ENCA, 2000);
    mbot->motor4 = bimotor_init(4, BRIDGE2_ENB, BRIDGE2_IN3, BRIDGE2_IN4, BRIDGE2_ENCB, 2000);
    
    return mbot;
}

void mecanumbot_destroy(struct Mecanumbot *this){
    set_off(this);
    bimotor_destroy(this->motor1);
    bimotor_destroy(this->motor2);
    bimotor_destroy(this->motor3);
    bimotor_destroy(this->motor4);
    free(this);
}

void set_off(struct Mecanumbot *this)
{
    set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
    set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
    // set_motor_off();
    // set_motor_off();
    // set_motor_off();
    // set_motor_off();
}

void set_direction(struct Mecanumbot *this, double linear_x, double angular_z)
{
    if(linear_x > 0 && angular_z == 0)
        this->direction = MECANUMBOT_DIRECTION_FORWARD;
    else if(linear_x < 0 && angular_z == 0)
        this->direction = MECANUMBOT_DIRECTION_BACKWARD;
    else if(linear_x == 0 && angular_z > 0)
        this->direction = MECANUMBOT_DIRECTION_RIGHT;
    else if(linear_x == 0 && angular_z < 0)
        this->direction = MECANUMBOT_DIRECTION_LEFT;
    else if(linear_x > 0 && angular_z > 0)
        this->direction = MECANUMBOT_DIRECTION_FORWARD_RIGHT;
    else if(linear_x > 0 && angular_z < 0)
        this->direction = MECANUMBOT_DIRECTION_FORWARD_LEFT;
    else if(linear_x < 0 && angular_z > 0)
        this->direction = MECANUMBOT_DIRECTION_BACKWARD_RIGHT;
    else if(linear_x < 0 && angular_z < 0)
        this->direction = MECANUMBOT_DIRECTION_BACKWARD_LEFT;
        
    double x_vel = fmin(fabs(linear_x*MAX_VEL), MAX_VEL);
    double z_vel = fmin(fabs(angular_z*MAX_VEL), MAX_VEL);

    // printf("x: %.2f, y: %.2f\n\r", x_vel, z_vel);

    x_vel = x_vel >= MIN_VEL? x_vel: 0.0;
    z_vel = z_vel >= MIN_VEL? z_vel: 0.0;

    switch (this->direction)
    {
        case MECANUMBOT_DIRECTION_FORWARD:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_BACKWARD:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
            break;
        case MECANUMBOT_DIRECTION_RIGHT:
            set_motor_speed(this->motor1, z_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, z_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor3, z_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, z_vel, BIMOTOR_BACKWARD);
            break;
        case MECANUMBOT_DIRECTION_LEFT:
            set_motor_speed(this->motor1, z_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor2, z_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, z_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor4, z_vel, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_FORWARD_RIGHT:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_FORWARD_LEFT:
            set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_BACKWARD_RIGHT:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor2, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor4, 0, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_BACKWARD_LEFT:
            set_motor_speed(this->motor1, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor3, 0, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
            break;
        case MECANUMBOT_DIRECTION_CLOCKWISE:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_FORWARD);
            break;
        case MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE:
            set_motor_speed(this->motor1, x_vel, BIMOTOR_BACKWARD);
            set_motor_speed(this->motor2, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor3, x_vel, BIMOTOR_FORWARD);
            set_motor_speed(this->motor4, x_vel, BIMOTOR_BACKWARD);
            break;
        default:
            break;
    }
}