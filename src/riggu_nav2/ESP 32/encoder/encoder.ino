#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32.h>

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 25

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 26

// Motor control pins 
#define PWM_PIN_LEFT 19
#define PWM_PIN_RIGHT 35
#define DIR_LEFT 34
#define DIR_RIGHT 32


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher;
rcl_publisher_t tickPub;
rcl_publisher_t ratePub_r;
rcl_publisher_t ratePub_l;
rcl_subscription_t subl;
rcl_subscription_t subr;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist tick_count;
std_msgs__msg__Float32 rate_l;
std_msgs__msg__Float32 rate_r;
std_msgs__msg__Float32 toggle_msgl;
std_msgs__msg__Float32 toggle_msgr;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;
rcl_node_t node;
Adafruit_MPU6050 mpu;


// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
int pwm_got_l;
int pwm_got_r;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
float current_ticks_l = 0 ;
float current_ticks_r = 0 ;

long prev_ticks_l=0;
long prev_ticks_r=0;


void motor_pwr_left()          // To Control the direction and power of  the left motor
{
  if(pwm_got_l>0)
  {           
    digitalWrite(DIR_LEFT,HIGH);
  }
  else if(pwm_got_l<0)
  {
    digitalWrite(DIR_LEFT,LOW);
  }
  else
  {
    digitalWrite(DIR_LEFT,LOW);
  }
  analogWrite(PWM_PIN_LEFT,abs(pwm_got_l));
 }

void motor_pwr_right()          // To Control the direction and power of  the right motor
{
  if(pwm_got_r>0)
  {           
    digitalWrite(DIR_RIGHT,HIGH);
  }
  else if(pwm_got_r<0)
  {
    digitalWrite(DIR_RIGHT,LOW);
  }
  else
  {
    digitalWrite(DIR_RIGHT,LOW);
  }
  analogWrite(PWM_PIN_RIGHT,abs(pwm_got_r));
}


void right_wheel_tick()         // Increment the number of ticks
{
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (tick_count.linear.y== encoder_maximum) {
      tick_count.linear.y= encoder_minimum;
    }
    else {
      tick_count.linear.y++;  
    }    
  }
  else {
    if (tick_count.linear.y== encoder_minimum) {
      tick_count.linear.y= encoder_maximum;
    }
    else {
      tick_count.linear.y--;  
    }   
  }
}


void left_wheel_tick()            // Increment the number of ticks
{
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (tick_count.linear.x== encoder_maximum) {
      tick_count.linear.x= encoder_minimum;
    }
    else {
      tick_count.linear.x++;  
    }  
  }
  else {
    if (tick_count.linear.x== encoder_minimum) {
      tick_count.linear.x= encoder_maximum;
    }
    else {
      tick_count.linear.x--;  
    }   
  }
}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void messageCbl(const void * msgin)
{  
  const std_msgs__msg__Float32 * toggle_msgl = (const std_msgs__msg__Float32 *)msgin;
  pwm_got_l=(int)toggle_msgl->data;  
}

void messageCbr(const void * msgin)
{  
  const std_msgs__msg__Float32 * toggle_msgr = (const std_msgs__msg__Float32 *)msgin;
  pwm_got_l=(int)toggle_msgr->data;   
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&tickPub, &tick_count, NULL));
    RCSOFTCHECK(rcl_publish(&ratePub_l, &rate_l, NULL));
    RCSOFTCHECK(rcl_publish(&ratePub_r, &rate_r, NULL));
    
  }
}

void setup() {
  Serial.begin(115200);
  //set_microros_transports();
  // set_microros_wifi_transports("Pixel", "12345678", "192.168.12.1", 8888);
  set_microros_transport();

  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "sensor_msgs/Imu"));

  RCCHECK(rclc_publisher_init_default(
    &tickPub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "ticks_count"));

  RCCHECK(rclc_publisher_init_default(
    &ratePub_r,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "tickrate/right"));

  RCCHECK(rclc_publisher_init_default(
    &ratePub_l,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "tickrate/left"));

  RCCHECK(rclc_subscription_init_default(
    &subl,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "left_wheel/control_effort"));

  RCCHECK(rclc_subscription_init_default(
    &subr,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "right_wheel/control_effort"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subl, &toggle_msgl, &messageCbl, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subr, &toggle_msgr, &messageCbr, ON_NEW_DATA));

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  pinMode(PWM_PIN_LEFT, OUTPUT);
  pinMode(PWM_PIN_RIGHT, OUTPUT);
  pinMode(DIR_LEFT,OUTPUT);
  pinMode(DIR_RIGHT,OUTPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

}


void loop() {
 
 sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /*Saves IMU Data into respective variables*/

  imu_msg.linear_acceleration.x = double(a.acceleration.x);
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  imu_msg.linear_acceleration.y = double(a.acceleration.y);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  imu_msg.linear_acceleration.z = double(a.acceleration.z);
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  imu_msg.angular_velocity.x = double(g.gyro.x);
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  imu_msg.angular_velocity.y = double(g.gyro.y);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  imu_msg.angular_velocity.z = double(g.gyro.z);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  imu_msg.orientation.w = double(temp.temperature);
  Serial.print(temp.temperature);
  Serial.println(" degC");


  // Record the time
  currentMillis = millis();

   
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
    current_ticks_l = tick_count.linear.x;
    current_ticks_r = tick_count.linear.y;

    rate_l.data =  (current_ticks_l-prev_ticks_l)*1000/interval;
    rate_r.data =  (current_ticks_r-prev_ticks_r)*1000/interval;

    previousMillis = currentMillis;
    prev_ticks_l = current_ticks_l; 
    prev_ticks_r = current_ticks_r;

    RCSOFTCHECK(rcl_publish(&tickPub, &tick_count, NULL));
    RCSOFTCHECK(rcl_publish(&ratePub_l, &rate_l, NULL));
    RCSOFTCHECK(rcl_publish(&ratePub_r, &rate_r, NULL));
    motor_pwr_left();
    motor_pwr_right();
  };
  RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
}
