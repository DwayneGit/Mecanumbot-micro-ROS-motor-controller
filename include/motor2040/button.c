#include <stdint.h>
#include "pico/stdlib.h"
#include "motor2040/button.h"

struct Button * button_init(uint pin, enum Polarity polarity, uint32_t repeat_time, uint32_t hold_time){
    
    struct Button * btn;
    btn = (struct Button *) malloc(sizeof(struct Button));

    btn->pin = pin;
    btn->polarity = polarity;
    btn->repeat_time = repeat_time;
    btn->hold_time = hold_time;
    btn->pressed = false;
    btn->last_state = false;
    btn->pressed_time = 0;
    btn->last_time = 0;
    
    gpio_set_function(pin, GPIO_FUNC_SIO);
    gpio_set_dir(pin, GPIO_IN);

    if (polarity == ACTIVE_LOW) {
        gpio_pull_up(pin);
    }
    else {
        gpio_pull_down(pin);
    }

    return btn;
};

struct Button * button_init_default(uint pin){
    return button_init(pin, ACTIVE_LOW, 200, 1000);
}

    
bool button_raw(struct Button *this) {
    if(this->polarity == ACTIVE_LOW){
    return !gpio_get(this->pin);
    } else {
    return gpio_get(this->pin);
    }
}

bool button_read(struct Button *this) {
    auto time = millis();
    bool state = button_raw(this);
    bool changed = state != this->last_state;
    this->last_state = state;

    if(changed) {
        if(state) {
            this->pressed_time = time;
            this->pressed = true;
            this->last_time = time;
            return true;
        }
        else {
            this->pressed_time = 0;
            this->pressed = false;
            this->last_time = 0;
        }
    }
    // Shortcut for no auto-repeat
    if(this->repeat_time == 0) return false;

    if(this->pressed) {
        uint32_t repeat_rate = this->repeat_time;
        if(this->hold_time > 0 && time - this->pressed_time > this->hold_time) {
            repeat_rate /= 3;
        }
        if(time - this->last_time > repeat_rate) {
            this->last_time = time;
            return true;
        }
    }

    return false;
}