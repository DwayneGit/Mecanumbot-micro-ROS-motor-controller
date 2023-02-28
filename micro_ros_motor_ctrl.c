#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "include/micro_ros_motor_ctrl/mecanumbot.h"
#include "include/micro_ros_motor_ctrl/pico_uart_transports.h"

#define BRIDGE1_ENA 8 
#define BRIDGE1_IN1 7
#define BRIDGE1_IN2 6
#define BRIDGE1_ENCA 18

#define BRIDGE1_ENB 2
#define BRIDGE1_IN3 4
#define BRIDGE1_IN4 3
#define BRIDGE1_ENCB 19

#define BRIDGE2_ENA 10
#define BRIDGE2_IN1 11
#define BRIDGE2_IN2 12
#define BRIDGE2_ENCA 20

#define BRIDGE2_ENB 15
#define BRIDGE2_IN3 13
#define BRIDGE2_IN4 14
#define BRIDGE2_ENCB 21

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n\r",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n\r",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_publisher_t pubFrontRightTicks;
rcl_publisher_t pubFrontLeftTicks;
rcl_publisher_t pubRearRightTicks;
rcl_publisher_t pubRearLeftTicks;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&pubFrontRightTicks, &mecanumbot->motor1->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubFrontLeftTicks, &mecanumbot->motor2->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubRearRightTicks, &mecanumbot->motor3->encoderTickCount, NULL));
        RCSOFTCHECK(rcl_publish(&pubRearLeftTicks,  &mecanumbot->motor4->encoderTickCount, NULL));
	}
}

void subscription_callback(const void * msgin)
{
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
    set_direction(mecanumbot, msg->linear.x, msg->angular.z);
}

void wheel1_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE1_ENCA) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor1);
    }
}

void wheel2_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE1_ENCB) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor2);
    }
}

void wheel3_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE2_ENCA) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor3);
    }
}

void wheel4_enc_tick(void){
    if (gpio_get_irq_event_mask(BRIDGE2_ENCB) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE);
        get_encoder_data(mecanumbot->motor4);
    }
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // // Turn off FIFO's - we want to do this character by character
    // uart_set_fifo_enabled(UART_ID, false);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

    gpio_add_raw_irq_handler(BRIDGE1_ENCA, wheel1_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE1_ENCB, wheel2_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCA, wheel3_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCB, wheel4_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    // multicore_launch_core1(core1_entry);

    mecanumbot = mecanumbot_init();

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    RCCHECK(rmw_uros_ping_agent(timeout_ms, attempts));

    // Node 0 - Motor Controller
    rcl_node_t node0;
    RCCHECK(rclc_node_init_default(&node0, "motor_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node0,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

	rclc_executor_t executor0 = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor0, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor0, 
        &subscriber, 
        &twist_msg,
        &subscription_callback, 
        ON_NEW_DATA
    ));

    // Node 1 - Wheel Encoder Ticks
    rcl_node_t node1;
    RCCHECK(rclc_node_init_default(&node1, "wheel_encoder_ticks", "", &support));
     
    (rclc_publisher_init_default(&pubFrontRightTicks, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_right_ticks"));
    (rclc_publisher_init_default(&pubFrontLeftTicks, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_left_ticks"));
    (rclc_publisher_init_default(&pubRearRightTicks, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_right_ticks"));
    (rclc_publisher_init_default(&pubRearLeftTicks, &node1, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_left_ticks"));

    rcl_timer_t timer;
    const unsigned int timer_period = RCL_MS_TO_NS(100);
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

    rclc_executor_t executor1 = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(
        &executor1,
        &support.context, 
        2, 
        &allocator
    ));
    RCCHECK(rclc_executor_add_timer(&executor1, &timer));
    
    // set_on(mecanumbot);

    // rclc_executor_spin(&executor);
    while (true) {
        rclc_executor_spin_some(&executor0, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor1, RCL_MS_TO_NS(100));
    }

    mecanumbot_destroy(mecanumbot);
    RCCHECK(rclc_executor_fini(&executor0));
    RCCHECK(rclc_executor_fini(&executor1));
    RCCHECK(rcl_subscription_fini(&subscriber, &node0));
    RCCHECK(rcl_publisher_fini(&pubFrontRightTicks, &node1));   
    RCCHECK(rcl_publisher_fini(&pubFrontLeftTicks, &node1));   
    RCCHECK(rcl_publisher_fini(&pubRearRightTicks, &node1));   
    RCCHECK(rcl_publisher_fini(&pubRearLeftTicks,  &node1));    
    RCCHECK(rcl_node_fini(&node0));
    RCCHECK(rcl_node_fini(&node1));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_support_fini(&support))
    
    return 0;
}
