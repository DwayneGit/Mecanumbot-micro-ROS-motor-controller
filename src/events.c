#include "../include/micro_ros_motor_ctrl/events.h"

uint32_t gpio_get_events(uint gpio){
    int32_t mask = 0xF << 4 * (gpio % 8);
    return (iobank0_hw->intr[gpio /8] & mask) >> 4 * (gpio % 8);
}

void gpio_clear_events(uint gpio, uint32_t events){
    gpio_acknowledge_irq(gpio, events);
}
