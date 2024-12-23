#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/int16.h>



geometry_msgs__msg__Quaternion msg;
std_msgs__msg__Int16 msg1;
std_msgs__msg__Int16 msg2;


rcl_subscription_t subscriber;

rcl_subscription_t subscriber_lift;
rcl_subscription_t subscriber_piston;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;  


// 21 isn't a great pin for servo control

const int dir_pin1 = 22;
const int pwm_pin1 = 23;

const int dir_pin2 = 25;
const int pwm_pin2 = 26;

const int dir_pin3 = 32;
const int pwm_pin3 = 33;

const int motor_lift_dir= 35;
const int motor_lift_pwm = 34; 

const int piston_pin = 25;




int dirn = 0, vel = 0, m1 = 0, m2 = 0, m3 = 0, m4 = 0 ;
int piston = 0, piston_lift_sate = 0;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }
 
void error_loop() {
  while (1) {
    delay(100);
  }
}
 
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Quaternion *msg = (const geometry_msgs__msg__Quaternion *)msgin;
     m1 = (int)msg->x;
     m2 = (int)msg->y;
     m3 = (int)msg->z;
     
   
}

void grpr_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  
    m4 = (int)msg->data;
     
     
} 

void piston_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  piston = (int)msg->data;
     if(piston)
     {
      digitalWrite(piston_pin,HIGH);
     } 
     else 
     {

      digitalWrite(piston_pin,LOW);
     }
     
     
    
     

} 

void setup() {
  set_microros_wifi_transports("robocon24", "12345678", "192.168.122.245", 8888);
  //set_microros_transports();
  pinMode(pwm_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);

  pinMode(pwm_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);

  pinMode(pwm_pin3, OUTPUT);
  
  pinMode(dir_pin3, OUTPUT);

  pinMode(piston_pin, OUTPUT);
 

  pinMode(motor_lift_dir, OUTPUT);
  pinMode(motor_lift_pwm, OUTPUT);
  
 
  allocator = rcl_get_default_allocator();
 
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
 
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
      "motor_vel"));
  
  RCCHECK(rclc_subscription_init_default(
    &subscriber_lift,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "lift")); 
 
  RCCHECK(rclc_subscription_init_default(
    &subscriber_piston,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "piston")); 

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_lift, &msg1, &grpr_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_piston, &msg2, &piston_subscription_callback, ON_NEW_DATA));
  

}
 
void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  motor(m1, dir_pin1, pwm_pin1);
  motor(m2, dir_pin2, pwm_pin2);
  motor(m3, dir_pin3, pwm_pin3);
  motor(m4, motor_lift_dir, motor_lift_pwm);
  
  
  }
 
// Function to move the robot forward
void motor(int speed, int dirpin, int pwmpin) {
  if (speed > 0)
    digitalWrite(dirpin, 1);
  
  else
  {
    digitalWrite(dirpin, 0);
  }
  analogWrite(pwmpin, abs(speed));
}
