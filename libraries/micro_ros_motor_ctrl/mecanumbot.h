#ifndef MECANUMBOT_HPP
#define MECANUMBOT_HPP

#include <stdio.h>
#include "bimotor.h"

// Motor 1 GPIO
#define BRIDGE1_ENB 2
#define BRIDGE1_IN3 4
#define BRIDGE1_IN4 5
#define BRIDGE1_ENCB 19

// Motor 2 GPIO
#define BRIDGE1_ENA 8 
#define BRIDGE1_IN1 7
#define BRIDGE1_IN2 6
#define BRIDGE1_ENCA 18

// Motor 3 GPIO
#define BRIDGE2_ENA 10
#define BRIDGE2_IN1 8
#define BRIDGE2_IN2 9
#define BRIDGE2_ENCA 20

// Motor 4 GPIO
#define BRIDGE2_ENB 15
#define BRIDGE2_IN3 10
#define BRIDGE2_IN4 11
#define BRIDGE2_ENCB 21

// Directions 
#define MECANUMBOT_DIRECTION_FORWARD 0
#define MECANUMBOT_DIRECTION_BACKWARD 1
#define MECANUMBOT_DIRECTION_RIGHT 2
#define MECANUMBOT_DIRECTION_LEFT 3
#define MECANUMBOT_DIRECTION_FORWARD_RIGHT 4
#define MECANUMBOT_DIRECTION_FORWARD_LEFT 5
#define MECANUMBOT_DIRECTION_BACKWARD_RIGHT 6 
#define MECANUMBOT_DIRECTION_BACKWARD_LEFT 7
#define MECANUMBOT_DIRECTION_CLOCKWISE 8
#define MECANUMBOT_DIRECTION_COUNTER_CLOCKWISE 9

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define MAX_VEL 100.0
#define MIN_VEL 45.0

static int chars_rxed = 0;

struct Mecanumbot
{
    bool on;
    uint direction;
    struct BiMotor * motor1;
    struct BiMotor * motor2;
    struct BiMotor * motor3;
    struct BiMotor * motor4;
};

void set_off(struct Mecanumbot *this);
void mecanumbot_destroy(struct Mecanumbot *this);
void set_direction(struct Mecanumbot *this, double linear_x, double angular_z);

struct Mecanumbot * mecanumbot_init();

#endif