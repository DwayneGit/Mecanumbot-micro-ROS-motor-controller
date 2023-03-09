
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <micro_ros_utilities/type_utilities.h>
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "include/micro_ros_motor_ctrl/mecanumbot.h"
#include "include/micro_ros_motor_ctrl/pico_uart_transports.h"

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#define FLAG_VALUE 123
#define PUBLISHER_NUMBER 4

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n\r",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n\r",__LINE__,(int)temp_rc);}}

struct Mecanumbot *mecanumbot;

rcl_node_t motor_ctrlr_node;
rcl_node_t wheel_ticks_node;
rcl_subscription_t subscriber;
rcl_publisher_t enc_pub;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t motor_ctrlr_executor;
rclc_executor_t wheel_ticks_executor;

queue_t node1_queue;

geometry_msgs__msg__Twist twist_msg;

bool micro_ros_init_successful;
volatile bool exit_flag = false;

typedef struct {
    struct Mecanumbot * mecanumbot;
    rcl_publisher_t * enc_pub;
    int flag;
} queue_node_req_t;

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

void create_entities()
{	
    int rc_check = RCL_RET_OK;

    allocator = rcl_get_default_allocator();
	motor_ctrlr_executor = rclc_executor_get_zero_initialized_executor();

    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    /*----------------------------- Motor Controler Node ----------------------------------*/

    RCSOFTCHECK(rclc_node_init_default(&motor_ctrlr_node, "motor_ctrlr", "", &support));

    RCSOFTCHECK(rclc_subscription_init_default(
        &subscriber,
        &motor_ctrlr_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    RCSOFTCHECK(rclc_executor_init(&motor_ctrlr_executor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_subscription(&motor_ctrlr_executor, &subscriber, &twist_msg, &subscription_callback, ON_NEW_DATA));

    const uint64_t timeout_ns = 100000000;
    RCSOFTCHECK(rclc_executor_set_timeout(&motor_ctrlr_executor, timeout_ns));

    /*------------------ Wheel Encoder Ticks Node - Odometry -----------------------------*/

    RCSOFTCHECK(rclc_node_init_default(&wheel_ticks_node, "wheel_encoders", "", &support));
    
    RCSOFTCHECK(rclc_publisher_init_default(
        &enc_pub, 
        &wheel_ticks_node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), 
        "encoders"));

    RCSOFTCHECK(rclc_executor_init(&wheel_ticks_executor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_set_timeout(&wheel_ticks_executor, timeout_ns));

    mecanumbot = mecanumbot_init();

    queue_init(&node1_queue, sizeof(queue_node_req_t), 2);

	micro_ros_init_successful = true;
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void destroy_entities()
{
	exit_flag = true;
    mecanumbot_destroy(mecanumbot);
    queue_free(&node1_queue);

    RCSOFTCHECK(rclc_executor_fini(&wheel_ticks_executor));
    RCSOFTCHECK(rclc_executor_fini(&motor_ctrlr_executor));
    RCSOFTCHECK(rcl_subscription_fini(&subscriber, &motor_ctrlr_node));
	RCSOFTCHECK(rcl_publisher_fini(&enc_pub, &wheel_ticks_node));
    RCSOFTCHECK(rclc_support_fini(&support))
    RCSOFTCHECK(rcl_node_fini(&motor_ctrlr_node));
    RCSOFTCHECK(rcl_node_fini(&wheel_ticks_node));
    
	micro_ros_init_successful = false;
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
}

void core1_entry() {
	queue_node_req_t args;
    std_msgs__msg__Int32MultiArray msg;
    std_msgs__msg__Int32MultiArray__init(&msg);
    std_msgs__msg__MultiArrayDimension__Sequence__init(&msg.layout.dim, 1);
    
    rosidl_runtime_c__String label;
    label.data = "encoder_ticks";
    msg.layout.dim.data[0].label = label;
    msg.layout.dim.data[0].size = 4;
    msg.layout.dim.data[0].stride = 1;
    msg.layout.data_offset = 0;
    rosidl_runtime_c__int32__Sequence__init(&msg.data, 4);

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, true);

    gpio_add_raw_irq_handler(BRIDGE1_ENCA, wheel1_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE1_ENCB, wheel2_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCA, wheel3_enc_tick);
    gpio_add_raw_irq_handler(BRIDGE2_ENCB, wheel4_enc_tick);
    
    irq_set_enabled(IO_IRQ_BANK0, true);
    
    multicore_fifo_push_blocking(FLAG_VALUE);
    printf("Waiting for core 0...\n\r");

    queue_remove_blocking(&node1_queue, &args);
    rcl_publisher_t * enc_pub = args.enc_pub;
    struct Mecanumbot * mbot = args.mecanumbot;
    int id = args.flag;

    if (id != FLAG_VALUE)
        printf("Hmm, that's not right on core 1!\n\r");
    else {
        uint32_t period_us =  1e6 + ((rand()) % 10) * 1e5;
        while (!exit_flag) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);

            msg.data.data[0] = mbot->motor1->encoderTickCount;
            msg.data.data[1] = mbot->motor2->encoderTickCount;
            msg.data.data[2] = mbot->motor3->encoderTickCount;
            msg.data.data[3] = mbot->motor4->encoderTickCount;
            
            RCSOFTCHECK(rcl_publish(enc_pub, &msg, NULL));
            usleep(period_us);
        }
    }

    gpio_set_irq_enabled(BRIDGE1_ENCA, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(BRIDGE1_ENCB, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(BRIDGE2_ENCA, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(BRIDGE2_ENCB, GPIO_IRQ_EDGE_RISE, false);
    gpio_remove_raw_irq_handler(BRIDGE1_ENCA, wheel1_enc_tick);
    gpio_remove_raw_irq_handler(BRIDGE1_ENCB, wheel2_enc_tick);
    gpio_remove_raw_irq_handler(BRIDGE2_ENCA, wheel3_enc_tick);
    gpio_remove_raw_irq_handler(BRIDGE2_ENCB, wheel4_enc_tick);
    irq_set_enabled(IO_IRQ_BANK0, false);
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

    while (true)        
    {   
        const int timeout_ms = 2000; 
        const uint8_t attempts = 2;
        if (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts))
        {
            if (!micro_ros_init_successful) {
                create_entities();

                multicore_launch_core1(core1_entry);
                uint32_t g = multicore_fifo_pop_blocking();

                if (g != FLAG_VALUE){
                    printf("Hmm, that's not right on core 0!\n\r");
	                micro_ros_init_successful = false;
                }
                else {
                    queue_node_req_t args = {
                        mecanumbot,
                        &enc_pub,
                        FLAG_VALUE
                    };
                    queue_add_blocking(&node1_queue, &args);
                    rclc_executor_spin(&motor_ctrlr_executor);
                    rclc_executor_spin(&wheel_ticks_executor);
                }
            } else {
                rclc_executor_spin_some(&motor_ctrlr_executor, RCL_MS_TO_NS(10));
                rclc_executor_spin_some(&wheel_ticks_executor, RCL_MS_TO_NS(10));
            }
        } 
        else if (micro_ros_init_successful)
        {
          destroy_entities();
        }
    }

    return 0;
}
