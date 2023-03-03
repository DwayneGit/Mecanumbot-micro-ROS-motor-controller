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
#include "hardware/irq.h"

#include "include/micro_ros_motor_ctrl/mecanumbot.h"
#include "include/micro_ros_motor_ctrl/pico_uart_transports.h"

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

void wheel1_enc_tick(uint gpio, uint32_t events){
    if (gpio == BRIDGE1_ENCA)
        get_encoder_data(mecanumbot->motor1);
    else if (gpio == BRIDGE1_ENCB)
        get_encoder_data(mecanumbot->motor2);
    else if (gpio == BRIDGE2_ENCA)
        get_encoder_data(mecanumbot->motor3);
    else if (gpio == BRIDGE2_ENCB)
        get_encoder_data(mecanumbot->motor4);
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

    gpio_set_irq_callback(&wheel1_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    mecanumbot = mecanumbot_init();

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;
    RCCHECK(rmw_uros_ping_agent(timeout_ms, attempts));

    // Node 0 - Motor Controller
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "motor_controller", "", &support));

    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
     
    (rclc_publisher_init_default(&pubFrontRightTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_right_ticks"));
    (rclc_publisher_init_default(&pubFrontLeftTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "front_left_ticks"));
    (rclc_publisher_init_default(&pubRearRightTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_right_ticks"));
    (rclc_publisher_init_default(&pubRearLeftTicks, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "rear_left_ticks"));

    rcl_timer_t timer;
    const unsigned int timer_period = RCL_MS_TO_NS(100);
    RCCHECK(rclc_timer_init_default(&timer, &support, timer_period, timer_callback));

	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(
        &executor,
        &support.context, 
        2, 
        &allocator
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &subscriber, 
        &twist_msg,
        &subscription_callback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    set_on(mecanumbot);

    rclc_executor_spin(&executor);
    // while (true) {
    //     rclc_executor_spin_some(&executor0, RCL_MS_TO_NS(100));
    // }

    mecanumbot_destroy(mecanumbot);
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&pubFrontRightTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubFrontLeftTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubRearRightTicks, &node));   
    RCCHECK(rcl_publisher_fini(&pubRearLeftTicks,  &node));    
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rcl_timer_fini(&timer));
    RCCHECK(rclc_support_fini(&support))
    
    return 0;
}
