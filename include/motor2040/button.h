  
#include <stdint.h>
#include "pico/stdlib.h"
#include "motor2040/common.h"

struct Button {
    uint pin;
    enum Polarity polarity;
    uint32_t repeat_time;
    uint32_t hold_time;
    bool pressed;
    bool last_state;
    uint32_t pressed_time;
    uint32_t last_time;
};

struct Button * button_init(
    uint pin, 
    enum Polarity polarity, 
    uint32_t repeat_time, 
    uint32_t hold_time
);

struct Button * button_init_default(
    uint pin
);

bool button_raw(struct Button *this);
bool button_read(struct Button *this);
