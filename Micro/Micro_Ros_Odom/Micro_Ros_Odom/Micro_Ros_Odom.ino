// Include library micro-ROS untuk Arduino
#include <micro_ros_arduino.h>

// Include library ROS Client (RCL) dan Executor
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include tipe pesan ROS yang akan digunakan (Int32MultiArray)
#include <std_msgs/msg/int32_multi_array.h>

// Definisi pin untuk 4 encoder
#define ENCODER_PIN_A1 33
#define ENCODER_PIN_B1 32
#define ENCODER_PIN_A2 26
#define ENCODER_PIN_B2 25
#define ENCODER_PIN_A3 5
#define ENCODER_PIN_B3 18
#define ENCODER_PIN_A4 22
#define ENCODER_PIN_B4 23

// Variabel volatile untuk menyimpan data hitungan dari encoder
// Volatile diperlukan karena variabel ini diubah di dalam Interrupt Service Routine (ISR)
volatile long encoder1_count = 0;
volatile long encoder2_count = 0;
volatile long encoder3_count = 0;
volatile long encoder4_count = 0;

// Deklarasi objek-objek micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg; // Menggunakan tipe pesan Int32MultiArray
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Makro untuk penanganan error. Jika fungsi ROS gagal, panggil error_loop()
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Fungsi yang akan dipanggil jika terjadi error fatal pada koneksi micro-ROS
void error_loop(){
  Serial.println("Error in micro-ROS. Entering infinite loop.");
  while(1){
    delay(1000);
  }
}

// --- Interrupt Service Routines (ISR) untuk membaca Encoder ---
// ISR harus secepat mungkin. Hanya membaca pin dan mengubah variabel counter.

void IRAM_ATTR readEncoder1(){
  // Baca pin B untuk menentukan arah putaran
  if (digitalRead(ENCODER_PIN_B1) == HIGH) {
    encoder1_count++;
  } else {
    encoder1_count--;
  }
}

void IRAM_ATTR readEncoder2(){
  if (digitalRead(ENCODER_PIN_B2) == HIGH) {
    encoder2_count++;
  } else {
    encoder2_count--;
  }
}

void IRAM_ATTR readEncoder3(){
  if (digitalRead(ENCODER_PIN_B3) == HIGH) {
    encoder3_count++;
  } else {
    encoder3_count--;
  }
}

void IRAM_ATTR readEncoder4(){
  if (digitalRead(ENCODER_PIN_B4) == HIGH) {
    encoder4_count++;
  } else {
    encoder4_count--;
  }
}


// Callback function untuk timer, dijalankan setiap 1 detik
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Salin nilai counter encoder ke dalam pesan ROS
    // (Membaca variabel volatile dengan aman)
    noInterrupts(); // Nonaktifkan interrupt sementara untuk pembacaan yang aman
    msg.data.data[0] = encoder1_count;
    msg.data.data[1] = encoder2_count;
    msg.data.data[2] = encoder3_count;
    msg.data.data[3] = encoder4_count;
    interrupts(); // Aktifkan kembali interrupt

    // Publikasikan pesan
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  // Mulai komunikasi serial untuk debugging
  Serial.begin(115200);

  // Atur transport layer untuk micro-ROS (misal: Serial, WiFi, Ethernet)
  set_microros_transports();
  
  delay(2000);

  // --- Setup Encoder ---
  // Atur semua pin encoder sebagai INPUT_PULLUP
  pinMode(ENCODER_PIN_A1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B2, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A3, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B3, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A4, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B4, INPUT_PULLUP);

  // Pasang interrupt pada pin A dari setiap encoder
  // ISR akan dijalankan setiap kali ada perubahan sinyal (RISING and FALLING edge)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A2), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A3), readEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A4), readEncoder4, CHANGE);


  // --- Setup micro-ROS ---
  allocator = rcl_get_default_allocator();

  // Buat init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Buat node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_encoder_node", "", &support));

  // Buat publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), // Tipe pesan diubah
    "encoder_counts")); // Nama topik diubah

  // Buat timer, atur untuk berjalan setiap 100 ms (10 Hz)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Buat executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Inisialisasi pesan Int32MultiArray
  // Alokasikan memori untuk 4 integer
  msg.data.capacity = 4;
  msg.data.size = 4;
  msg.data.data = (int32_t*) malloc(msg.data.capacity * sizeof(int32_t));
  
  // Beri nilai awal 0 untuk semua elemen array
  for(int i = 0; i < 4; i++) {
    msg.data.data[i] = 0;
  }
}

void loop() {
  delay(10); // Penundaan singkat untuk stabilitas
  // Jalankan executor untuk memproses event seperti timer callback
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
