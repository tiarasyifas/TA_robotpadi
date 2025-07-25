[media pointer="file-service://file-SUUKw5ytn17a9A6YSdaRJn"]
[media pointer="file-service://file-SEsCaEboeNdZDXfukBiqJa"]
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#define LED_PIN 13
#define ENCODER_PIN_A 33  // Channel A
#define ENCODER_PIN_B 32  // Channel B

volatile int32_t encoder_count = 0;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Interrupt Service Routine for Channel A
void IRAM_ATTR encoderA_ISR() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);

  if (A == B) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data = encoder_count;  // Kirim nilai encoder
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Setup encoder pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderA_ISR, CHANGE);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Init micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Init node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Init publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_position"));

  // Init timer
  const unsigned int timer_timeout = 100;  // 100ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Init executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(10);  // Kurangi delay agar lebih responsif
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
