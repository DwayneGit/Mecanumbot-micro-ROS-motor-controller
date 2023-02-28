#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include "pico/multicore.h"

#define UART_TX_PIN 16
#define UART_RX_PIN 17

#define FLAG_VALUE 123

recursive_mutex_t count_mutex;
long count;

void increment_count()
{
	recursive_mutex_enter_blocking(&count_mutex);
    count += 1;
    sleep_ms(1000);
	recursive_mutex_exit(&count_mutex);
}

long get_count()
{
    long c;

    recursive_mutex_enter_blocking(&count_mutex);
	c = count;
    sleep_ms(1000);
    recursive_mutex_exit(&count_mutex);
	return (c);
}

void core1_entry() {
    multicore_fifo_push_blocking(FLAG_VALUE);

    uint32_t g = multicore_fifo_pop_blocking();
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n\r");
    else{
        while(1)
            printf("Count: %ld\n\r", get_count());
    }
}


int main(void){
    stdio_init_all();

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    recursive_mutex_init(&count_mutex);

    multicore_launch_core1(core1_entry);
    uint32_t g = multicore_fifo_pop_blocking();
    
    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n\r");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        while(1){
            increment_count();
            printf("Count Post Increment: %ld\n\r", count);
        }
    }
}