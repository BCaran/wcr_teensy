#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <PWMServo.h>

// Teensy 4.0 pins
#define ESC_pin1 3 // ESC1 pin
#define ESC_pin2 4 // ESC2 pin
#define ESC_pin3 5 // ESC3 pin
#define ESC_pin4 6 // ESC4 pin

PWMServo ESC1;
PWMServo ESC2;
PWMServo ESC3;
PWMServo ESC4;

int thrusterPWM = 0;

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    if(thrusterPWM > 0){
      for(int i = thrusterPWM; i > 0; i--){
        thrusterPWM--;
        ESC1.write(thrusterPWM);
        ESC2.write(thrusterPWM);
        ESC3.write(thrusterPWM);
        ESC4.write(thrusterPWM);
        delay(100);
      }
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  thrusterPWM = msg->data;
  ESC1.write(thrusterPWM);
  ESC2.write(thrusterPWM);
  ESC3.write(thrusterPWM);
  ESC4.write(thrusterPWM);

}

void setup() {
  set_microros_transports();
  
  ESC1.attach(ESC_pin1, 1000, 2000);
  ESC2.attach(ESC_pin2, 1000, 2000);
  ESC3.attach(ESC_pin3, 1000, 2000);
  ESC4.attach(ESC_pin4, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_teensy_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "thrustersPWM"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
