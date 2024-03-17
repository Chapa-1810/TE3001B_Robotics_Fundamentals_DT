#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Arduino.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

rcl_subscription_t subscriberSignal;
rcl_publisher_t publisherAngSpeed;

std_msgs__msg__Float32 msgAngS;
std_msgs__msg__Float32 msgSignal;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

rcl_node_t node;
rcl_timer_t timer1;
rcl_timer_t timer2;
rcl_timer_t timer3;


#define LED_PIN 12
#define PWM_PIN 27
#define ENCODER_PIN 32
#define IN1 25
#define IN2 26


//PID constants
const float maxSpeed = 8.38; //rpm, need to convert to rad/s
const int maxVoltage = 12; //V
const int T = 0.05; //100 ms
const float Kp = 1;
const float Ki = 2.948282; //toChange

//PWM constants
const int PWMFreq = 5000; /* 5 KHz */
const int PWMChannel = 0;
const int PWMResolution = 8;
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
uint16_t dutyCycle = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

volatile int prevEncoderState = 0;
volatile int ticks = 0;
void IRAM_ATTR updateEncoder() {
  ticks++;
}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
//CREATE SUBSCRIBER CALLBACK

int signalROS = 0;
int orientation = 1;
float percent = 0;

  //Obtain signal and change motor direction accordingly
  void subscription_callback(const void * msgin){  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    if(abs(msg->data) <= 1){
      signalROS = abs(msg->data)/msg->data;
      percent = abs(msg->data);
      if(signalROS > 0){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        orientation = 1;
      }
      else{
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        orientation = -1;
      }
    }
    
  }


const int ticksPerS = 2096;


unsigned long prevTime = 0;
float motorSpeed = 0;

//Calculate motor speed
void timer1_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    if(millis()- prevTime >= 100){
     motorSpeed = (6.283185)*((float)ticks / 2100.0) * (1000.0 / (millis() - prevTime)); // Calculation based on encoder resolution and time
      prevTime = millis();
      ticks = 0;
    }
    
  }
}





float wd = 0;
float wMeasured = 0;
float error = 0;
float lastError = 0;
float u = 0;


//PI
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    wMeasured = motorSpeed; //Obtain measured speed in volts
    wd = percent*maxSpeed; //Obtain reference voltage of the duty cycle.
    error = abs(wd) - wMeasured; //Obtain error between reference volts and volt measured
    u = Kp*error + Ki*T*(error + lastError); //calculate new input
    dutyCycle = map(u, 0, maxSpeed, 0, MAX_DUTY_CYCLE); //Map input corresponding to the PWM dutycycle
    ledcWrite(PWMChannel, dutyCycle); 
    if(lastError < 10) lastError += error; //Update error
    else lastError = 0;
    
    
    }
}

//Publish angular velocity according to motor speed and orientation.
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) { 
    msgAngS.data = error; 
    RCSOFTCHECK(rcl_publish(&publisherAngSpeed, &msgAngS, NULL));
  }
}

void setup() {
  set_microros_transports();
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(PWM_PIN, PWMChannel);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN),updateEncoder,RISING);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Controller", "", &support));
  
  //Publisher /angular_speed
  RCCHECK(rclc_publisher_init_default(
    &publisherAngSpeed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "angular_speed"));

  // subscriber /pwm_duty_cycle
  RCCHECK(rclc_subscription_init_default(
    &subscriberSignal,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_duty_cycle"));

  //timers
  const unsigned int timer1_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer1_timeout),
    timer1_callback));

  const unsigned int timer2_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));

  const unsigned int timer3_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer3,
    &support,
    RCL_MS_TO_NS(timer3_timeout),
    timer3_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));
  RCCHECK(rclc_executor_add_timer(&executor, &timer3));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriberSignal, &msgSignal, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
