#ifndef EVENTS_HPP
#define EVENTS_HPP

#include "pico/stdlib.h"
#include "hardware/structs/iobank0.h"

uint32_t gpio_get_events(uint gpio);
void gpio_clear_events(uint gpio, uint32_t events);

#endif