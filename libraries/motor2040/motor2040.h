#ifndef MOTOR2040_H_
#define MOTOR2040_H_

#include "pico/stdlib.h"
#include "motor.h"

#define MOTOR_A_P 4
#define MOTOR_A_N 5
#define MOTOR_B_P 6
#define MOTOR_B_N 7
#define MOTOR_C_P 8
#define MOTOR_C_N 9
#define MOTOR_D_P 10
#define MOTOR_D_N 11

#define ENCODER_A_A 0
#define ENCODER_A_B 1
#define ENCODER_B_A 2
#define ENCODER_B_B 3
#define ENCODER_C_A 12
#define ENCODER_C_B 13
#define ENCODER_D_A 14
#define ENCODER_D_B  15

const pin_pair * MOTOR_A; //  = pin_pair_init(MOTOR_A_P, MOTOR_A_N);
const pin_pair * MOTOR_B; //  = pin_pair_init(MOTOR_B_P, MOTOR_B_N);
const pin_pair * MOTOR_C; //  = pin_pair_init(MOTOR_C_P, MOTOR_C_N);
const pin_pair * MOTOR_D; //  = pin_pair_init(MOTOR_D_P, MOTOR_D_N);

#define NUM_MOTORS = 4

// Although encoder A and B channels are arbitrary, our MMME Encoders
// that accompany Motor2040 count down when the motors are diving in a
// positive direction, so these pin pairs are set as B and A instead
const pin_pair * ENCODER_A; // = pin_pair_init(ENCODER_A_B, ENCODER_A_A);
const pin_pair * ENCODER_B; // = pin_pair_init(ENCODER_B_B, ENCODER_B_A);
const pin_pair * ENCODER_C; // = pin_pair_init(ENCODER_C_B, ENCODER_C_A);
const pin_pair * ENCODER_D; // = pin_pair_init(ENCODER_D_B, ENCODER_D_A);

#define NUM_ENCODERS 4

#define TX_TRIG 16
#define RX_ECHO 17

#define LED_DATA 18
#define NUM_LEDS 1

#define USER_SW  23

#define ADC_ADDR_0 22
#define ADC_ADDR_1 24
#define ADC_ADDR_2 25

#define ADC0 26
#define ADC1 27
#define ADC2 28
#define SHARED_ADC 29 // The pin used for the board's sensing features

#define CURRENT_SENSE_A_ADDR 0b000
#define CURRENT_SENSE_B_ADDR 0b001
#define CURRENT_SENSE_C_ADDR 0b010
#define CURRENT_SENSE_D_ADDR 0b011
#define VOLTAGE_SENSE_ADDR 0b100
#define FAULT_SENSE_ADDR 0b101

#define SENSOR_1_ADDR 0b110
#define SENSOR_2_ADDR 0b111
#define NUM_SENSORS 2

#define SHUNT_RESISTOR 0.47f
#define CURRENT_GAIN 1
#define VOLTAGE_GAIN 3.9f / 13.9f
#define CURRENT_OFFSET -0.005f

#endif