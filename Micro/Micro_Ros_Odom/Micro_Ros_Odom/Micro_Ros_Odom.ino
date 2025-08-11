#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/empty.h>

// --- Encoder Pin Definitions ---
#define ENCODER_PIN_A1 33
#define ENCODER_PIN_B1 32
#define ENCODER_PIN_A2 22
#define ENCODER_PIN_B2 23
#define ENCODER_PIN_A3 5
#define ENCODER_PIN_B3 18
#define ENCODER_PIN_A4 26
#define ENCODER_PIN_B4 25

// --- Encoder Tick Counters ---
volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;
volatile int encoderCount3 = 0;
volatile int encoderCount4 = 0;

// --- ROS2 Node and Publisher/Subscriber ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

rcl_publisher_t encoder_pub1, encoder_pub2, encoder_pub3, encoder_pub4;
std_msgs__msg__Int32 encoder_msg1, encoder_msg2, encoder_msg3, encoder_msg4;

// --- Reset Encoder Subscriber ---
rcl_subscription_t reset_sub;
std_msgs__msg__Empty reset_msg;

// --- Helper Macros ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() {
  while (1) {
    delay(100);
  }
}


// --- Reset Topic Callback ---
void reset_callback(const void *msgin) {
  (void)msgin;
  reset_encoder();
}

// --- Timer Callback: Publishes Encoder Data ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    encoder_msg1.data = encoderCount1;
    encoder_msg2.data = encoderCount2;
    encoder_msg3.data = encoderCount3;
    encoder_msg4.data = encoderCount4;

    RCSOFTCHECK(rcl_publish(&encoder_pub1, &encoder_msg1, NULL));
    RCSOFTCHECK(rcl_publish(&encoder_pub2, &encoder_msg2, NULL));
    RCSOFTCHECK(rcl_publish(&encoder_pub3, &encoder_msg3, NULL));
    RCSOFTCHECK(rcl_publish(&encoder_pub4, &encoder_msg4, NULL));
  }
}

// --- Encoder Callback Functions ---
void IRAM_ATTR EncoderCallback1() {
  bool A = digitalRead(ENCODER_PIN_A1);
  bool B = digitalRead(ENCODER_PIN_B1);
  encoderCount1 += (A == B) ? 1 : -1;
}

void IRAM_ATTR EncoderCallback2() {
  bool A = digitalRead(ENCODER_PIN_A2);
  bool B = digitalRead(ENCODER_PIN_B2);
  encoderCount2 += (A == B) ? 1 : -1;
}

void IRAM_ATTR EncoderCallback3() {
  bool A = digitalRead(ENCODER_PIN_A3);
  bool B = digitalRead(ENCODER_PIN_B3);
  encoderCount3 += (A == B) ? 1 : -1;
}

void IRAM_ATTR EncoderCallback4() {
  bool A = digitalRead(ENCODER_PIN_A4);
  bool B = digitalRead(ENCODER_PIN_B4);
  encoderCount4 += (A == B) ? 1 : -1;
}

// --- Setup Function ---
void setup() {
  encoderCount1 = 0;
  encoderCount2 = 0;
  encoderCount3 = 0;
  encoderCount4 = 0;
  
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);  // pastikan koneksi siap

  // Init Encoder Pins
//  pinMode(ENCODER_PIN_A1, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_B1, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_A3, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_B3, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_A4, INPUT_PULLUP);
//  pinMode(ENCODER_PIN_B4, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A1), EncoderCallback1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A2), EncoderCallback2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A3), EncoderCallback3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A4), EncoderCallback4, CHANGE);

  // ROS2 Init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "encoder_reader_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&encoder_pub1, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder1"));
  RCCHECK(rclc_publisher_init_default(&encoder_pub2, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder2"));
  RCCHECK(rclc_publisher_init_default(&encoder_pub3, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder3"));
  RCCHECK(rclc_publisher_init_default(&encoder_pub4, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder4"));

  // Subscriber: reset_encoder
  RCCHECK(rclc_subscription_init_default(&reset_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "reset_encoder"));

  // Timer & Executor
  const unsigned int timer_timeout = 100; // ms
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_sub, &reset_msg, &reset_callback, ON_NEW_DATA));

  // Init encoder messages
  encoder_msg1.data = 0;
  encoder_msg2.data = 0;
  encoder_msg3.data = 0;
  encoder_msg4.data = 0;
}

// --- Loop Function ---
void loop() {
  delay(10); // delay ringan
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
