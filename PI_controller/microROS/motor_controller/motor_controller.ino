#define LED_PIN 12
#define PWM_PIN 27
#define ENCODER_PIN 32
#define IN1 25
#define IN2 26

#define FREQUENCY 5000
#define MOTOR_CHANNEL 0
#define RESOLUTION 8

#define SPEED_MEASURING_TIMEOUT 10
#define SPEED_PUBLISHER_TIMEOUT 10


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t pwm_subscriber;
rcl_publisher_t speed_publisher;
std_msgs__msg__Float32 pub_msg;
std_msgs__msg__Float32 sub_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t adc_timer;

const char* node_name = "Controller";
const char* publisher_topic = "angular_speed";
const char* subscriber_topic = "setpoint";

float motor_current_speed = 0.0;
float current_pwm = 0.0;
bool motor_direction = 0;

unsigned long encoderTicks = 0;
volatile int prevEncoderState = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void speed_publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pub_msg.data = motor_direction ? motor_current_speed : -motor_current_speed;
    RCSOFTCHECK(rcl_publish(&speed_publisher, &pub_msg, NULL));
  }
}

uint prev_speed_measuring_time = 0;
void speed_measuring_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motor_current_speed = ((encoderTicks/2100.0)/(millis()-prev_speed_measuring_time))*1000 * 2 * M_PI;
    prev_speed_measuring_time = millis();
    encoderTicks = 0;
  }
}


void pwm_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    current_pwm = msg->data;
    if (msg->data > 0) {
        motor_direction = 1;
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else if (msg->data < 0) {
        motor_direction = 0;
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    float pwm_pct = abs(msg->data);
    pwm_pct = pwm_pct > 1 ? 1 : pwm_pct;
    ledcWrite(MOTOR_CHANNEL, pwm_pct * 255);
}

void setup() {
    set_microros_transports();
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    
    digitalWrite(LED_PIN, HIGH);
    ledcSetup(MOTOR_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(PWM_PIN, MOTOR_CHANNEL);
    ledcWrite(MOTOR_CHANNEL, 0);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), updateEncoder, CHANGE);


    delay(2000);

    allocator = rcl_get_default_allocator();

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
    &speed_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    publisher_topic));
    // create timer for adc
    RCCHECK(rclc_timer_init_default(
        &adc_timer,
        &support,
        RCL_MS_TO_NS(SPEED_MEASURING_TIMEOUT),
       speed_measuring_callback));
    
    // create timer for publishing
    RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(SPEED_PUBLISHER_TIMEOUT),
    speed_publisher_timer_callback));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
    &pwm_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    subscriber_topic));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &adc_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &pwm_subscriber, &sub_msg, &pwm_subscription_callback, ON_NEW_DATA));

    pub_msg.data = 0;
}

void loop() {
    delay(5);
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

// Function to update encoder ticks
void updateEncoder() {
    int currEncoderState = digitalRead(ENCODER_PIN);
    if (prevEncoderState == 0 && currEncoderState == 1) {
        encoderTicks++;
    }
    prevEncoderState = currEncoderState;
}