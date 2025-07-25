#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define ENCODER_PIN_A1 33
#define ENCODER_PIN_B1 32
#define ENCODER_PIN_A2 26
#define ENCODER_PIN_B2 25
#define ENCODER_PIN_A3 5
#define ENCODER_PIN_B3 18
#define ENCODER_PIN_A4 2
#define ENCODER_PIN_B4 4

volatile int encoder_position1 = 0;
volatile int encoder_position2 = 0;
volatile int encoder_position3 = 0;
volatile int encoder_position4 = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Interrupt handler untuk Channel A pada Encoder 1
void encoder_ISR1() {
  if (digitalRead(ENCODER_PIN_B1) == HIGH) {
    encoder_position1++;
  } else {
    encoder_position1--;
  }
}

// Interrupt handler untuk Channel A pada Encoder 2
void encoder_ISR2() {
  if (digitalRead(ENCODER_PIN_B2) == HIGH) {
    encoder_position2++;
  } else {
    encoder_position2--;
  }
}

// Interrupt handler untuk Channel A pada Encoder 3
void encoder_ISR3() {
  if (digitalRead(ENCODER_PIN_B3) == HIGH) {
    encoder_position3++;
  } else {
    encoder_position3--;
  }
}

// Interrupt handler untuk Channel A pada Encoder 4
void encoder_ISR4() {
  if (digitalRead(ENCODER_PIN_B4) == HIGH) {
    encoder_position4++;
  } else {
    encoder_position4--;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data = encoder_position1;  // Kirim posisi encoder 1
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    msg.data = encoder_position2;  // Kirim posisi encoder 2
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    msg.data = encoder_position3;  // Kirim posisi encoder 3
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    msg.data = encoder_position4;  // Kirim posisi encoder 4
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  set_microros_transports();

  // Setup pin encoder A dan B untuk 4 encoder
  pinMode(ENCODER_PIN_A1, INPUT);
  pinMode(ENCODER_PIN_B1, INPUT);
  pinMode(ENCODER_PIN_A2, INPUT);
  pinMode(ENCODER_PIN_B2, INPUT);
  pinMode(ENCODER_PIN_A3, INPUT);
  pinMode(ENCODER_PIN_B3, INPUT);
  pinMode(ENCODER_PIN_A4, INPUT);
  pinMode(ENCODER_PIN_B4, INPUT);

  // Attach interrupt pada Channel A untuk setiap encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A1), encoder_ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A2), encoder_ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A3), encoder_ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A4), encoder_ISR4, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_odom"));

  // Create timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
