#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/empty.h>

// Publishers
rcl_publisher_t mission_start_publisher;
rcl_publisher_t mission_abort_publisher;
rcl_publisher_t estop_publisher;

rcl_subscription_t mission_start_subscriber;
rcl_subscription_t mission_abort_subscriber;
rcl_subscription_t estop_subscriber;

// Template Empty Message
std_msgs__msg__Empty msg;
std_msgs__msg__Empty sub_msg1;
std_msgs__msg__Empty sub_msg2;
std_msgs__msg__Empty sub_msg3;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool micro_ros_init_successful;

// LED Flash Rate Limiters
#define FLASH_RATE_LIMIT 60
unsigned long red_time;
unsigned long yellow_time;
unsigned long green_time;

#define ESTOP_PIN 3
#define GO_PIN 4
#define ABORT_PIN 2
#define FLASH_DURATION 50

#define LED_PIN 13
#define RED_PIN 11
#define YELLOW_PIN 6
#define GREEN_PIN 5

//#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}

//void error_loop() {
//  while (1) {
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//    delay(100);
//  }
//}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if (digitalRead(ESTOP_PIN) == LOW) {
      RCSOFTCHECK(rcl_publish(&estop_publisher, &msg, NULL));
    }
    else if (digitalRead(ABORT_PIN) == LOW) {
      RCSOFTCHECK(rcl_publish(&mission_abort_publisher, &msg, NULL));
    }
    else if (digitalRead(GO_PIN) == LOW) {
      RCSOFTCHECK(rcl_publish(&mission_start_publisher, &msg, NULL));
    }
  }
}

void subscription_mission_start_callback(const void * msgin)
{
  if(millis() - green_time > FLASH_RATE_LIMIT){
    digitalWrite(GREEN_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(GREEN_PIN, LOW);
    green_time = millis();
  }
}

void subscription_mission_abort_callback(const void * msgin)
{
   if(millis() - yellow_time > FLASH_RATE_LIMIT){
    digitalWrite(YELLOW_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(YELLOW_PIN, LOW);
    yellow_time = millis();
   }
}

void subscription_estop_callback(const void * msgin)
{
  if(millis() - red_time > FLASH_RATE_LIMIT){
    digitalWrite(RED_PIN, HIGH);
    delay(FLASH_DURATION);
    digitalWrite(RED_PIN, LOW);
    red_time = millis();
  }
}


bool create_entities() {
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "safety_button", "", &support));

  // create mission_start_publisher
  RCCHECK(rclc_publisher_init_default(
            &mission_start_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
            "/mission_start"));

  // create mission_start_publisher
  RCCHECK(rclc_publisher_init_default(
            &mission_abort_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
            "/mission_abort"));

  // create mission_start_publisher
  RCCHECK(rclc_publisher_init_default(
            &estop_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
            "/emergency_stop"));

  // create mission start subscriber to flash light
  RCCHECK(rclc_subscription_init_default(
    &mission_start_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/mission_start"));

  // create mission start subscriber to flash light
  RCCHECK(rclc_subscription_init_default(
    &mission_abort_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/mission_abort"));

  // create mission start subscriber to flash light
  RCCHECK(rclc_subscription_init_default(
    &estop_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/emergency_stop"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  // create executor (with 4 handles - timer + 3 subs)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &mission_start_subscriber, &sub_msg1, &subscription_mission_start_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &mission_abort_subscriber, &sub_msg2, &subscription_mission_abort_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &estop_subscriber, &sub_msg3, &subscription_estop_callback, ON_NEW_DATA));

  micro_ros_init_successful = true;
  return true;
}

void destroy_entities()
{
  rcl_publisher_fini(&mission_start_publisher, &node);
  rcl_publisher_fini(&mission_abort_publisher, &node);
  rcl_publisher_fini(&estop_publisher, &node);
  rcl_subscription_fini(&mission_start_subscriber, &node);
  rcl_subscription_fini(&mission_abort_subscriber, &node);
  rcl_subscription_fini(&estop_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void setup() {
  set_microros_transports();

  // Setup inputs and outputs
  pinMode(ESTOP_PIN, INPUT);
  pinMode(ABORT_PIN, INPUT);
  pinMode(GO_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  micro_ros_init_successful = false;

//  delay(2000);
}

static unsigned long long prev_connect_test_time;

void flash_leds() {
   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
   digitalWrite(RED_PIN, !digitalRead(RED_PIN));
   digitalWrite(YELLOW_PIN, !digitalRead(YELLOW_PIN));
   digitalWrite(GREEN_PIN, !digitalRead(GREEN_PIN));
}

void leds_off() {
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
}

void loop() 
{
    // check if the agent got disconnected at 10Hz
    if(micros() - prev_connect_test_time > 100000)
    {
        prev_connect_test_time = micros();
        // check if the agent is connected
        if(RMW_RET_OK == rmw_uros_ping_agent(50, 2))
        {
            
            // reconnect if agent got disconnected or haven't at all
            if (!micro_ros_init_successful) 
            {
              leds_off();
              create_entities();
            } 
        } 
        else if(micro_ros_init_successful)
        {
            // clean up micro-ROS components
            destroy_entities();
        } 
    }
    
    if(micro_ros_init_successful)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    } else {
        flash_leds();
    }

    delay(10);
}