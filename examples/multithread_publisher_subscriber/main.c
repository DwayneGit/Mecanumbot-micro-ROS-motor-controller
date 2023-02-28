#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/header.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "../../include/micro_ros_motor_ctrl/pico_uart_transports.h"
// #include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#define FLAG_VALUE 123

#define STRING_BUFFER_LEN 100
#define PUBLISHER_NUMBER 3

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

typedef struct  {
    rcl_publisher_t *publisher;
    int flag;
} arg_struct_t;

rcl_node_t node;
// pthread_t pub_thr[PUBLISHER_NUMBER];
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Header recv_msg;
static micro_ros_utilities_memory_conf_t conf = {0};

volatile bool exit_flag = false;

// Publish thread

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
	printf("Received %d-%d from %s\n\r", msg->stamp.sec, msg->stamp.nanosec, msg->frame_id.data);
}

queue_t publish_queue;
void publish()
{
	arg_struct_t args;

	// Create and allocate the publisher message
	std_msgs__msg__Header send_msg;

	bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), 
	&send_msg,
	conf);
    
    multicore_fifo_push_blocking(FLAG_VALUE);
    printf("Waiting for core 0...\n\r");

    queue_remove_blocking(&publish_queue, &args);
    rcl_publisher_t * publisher = args.publisher;
    int id = args.flag;
    printf("Got the flag: %d\n\r", id);

    if ( id != FLAG_VALUE ){
        printf("Hmm, that's not right on core 1!\n\r");
		return;
    }
	else if (!success){
		printf("Error allocating message memory for publisher %d\n\r", id);
		return;
	}
    else {
        char message[STRING_BUFFER_LEN];
        sprintf(message, "Thread %d\n\r", id);
        send_msg.frame_id = micro_ros_string_utilities_set(send_msg.frame_id, message);

        uint32_t period_us =  1e6 + ((rand()) % 10) * 1e5;
        printf("Thread %d start, publish period: %.1f seconds\n\r", id, period_us/1000000.0);

        while (!exit_flag)
        {
            // Fill the message timestamp
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            send_msg.stamp.sec = ts.tv_sec;
            send_msg.stamp.nanosec = ts.tv_nsec;

            RCSOFTCHECK(rcl_publish(publisher, &send_msg, NULL));
            printf("Thread %d sent: %d-%d\n\r", id, send_msg.stamp.sec, send_msg.stamp.nanosec);	
            usleep(period_us);
        }
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

    gpio_set_function(16, GPIO_FUNC_UART);
    gpio_set_function(17, GPIO_FUNC_UART);

    queue_init(&publish_queue, sizeof(arg_struct_t), 2);
 
  	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// Create node
	RCCHECK(rclc_node_init_default(&node, "multithread_node", "", &support));

	// Create publishers
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
        "multithread_topic"));

	// Create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
		"multithread_topic"));
	
	// Configure and allocate the subscriber message
	conf.max_string_capacity = STRING_BUFFER_LEN;

	bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), 
	&recv_msg,
	conf);

	if (!success)
	{
		printf("Error allocating message memory for subscriber\n\r");
		return 1;
	}

	// Create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    // pthread_create(&pub_thr, NULL, publish, args);
    multicore_launch_core1(publish);
    
    uint32_t g = multicore_fifo_pop_blocking();
    if (g != FLAG_VALUE){
        printf("Hmm, that's not right on core 0!\n\r");
        return 1;    
    }
    else {
        // Start publish thread
        arg_struct_t args = {&publisher, FLAG_VALUE};
        queue_add_blocking(&publish_queue, &args);
        printf("Core 1 is ready!\n\r");

        // Set executor timeout to 100 ms to reduce thread locking time
        const uint64_t timeout_ns = 100000000;
        RCCHECK(rclc_executor_set_timeout(&executor,timeout_ns));
        rclc_executor_spin(&executor);
    }

	exit_flag = true;

	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));
}