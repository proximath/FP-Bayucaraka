#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#define STEP_PIN1 18
#define DIR_PIN1 4
#define STEP_PIN2 19
#define DIR_PIN2 21

long currentStepX = 0;
long currentStepY = 0;

Servo servoGrip;
int servoPinGrip = 22;
int pos = 0;

rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

std_msgs__msg__Int32MultiArray msg;
int32_t data_buffer[4];


void move_callback(const void * message){
  const std_msgs__msg__Int32MultiArray *msg = (const std_msgs__msg__Int32MultiArray *)message;

  if (msg->data.size < 4) return;

  int payX = msg->data.data[0];
  int payY = msg->data.data[1];
  int dropX = msg->data.data[2];
  int dropY = msg->data.data[3];

  Serial.println("================================");
  
  // TITIK PAYLOAD
  Serial.println("Step 1: Menuju Payload...");
  pindahKeKoordinat(payX, payY);

  // TUTUP GRIP
  for (pos = 0; pos <= 85; pos += 1) {
    servoGrip.write(pos);
    delay(20);
  }
  delay(500);
  servoGrip.detach();

  // TITIK DROPPING
  Serial.println("Step 2: Menuju Dropping...");
  pindahKeKoordinat(dropX, dropY);
  delay(2000);

  // BUKA GRIP
  servoGrip.attach(servoPinGrip, 600, 2400);
  for (pos = 85; pos >= 0; pos -= 1) {
    servoGrip.write(pos);
    delay(20);
  }
  delay(500);

  // KEMBALI KE TITIK AWAL (0,0)
  Serial.println("Step 3: Kembali ke Titik Awal (0,0)...");
  pindahKeKoordinat(0, 0);

  Serial.println("Siklus Selesai.");
  Serial.println("================================");
}


// ================= SETUP =================
void setup() {
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  Serial.begin(115200);

  servoGrip.attach(servoPinGrip, 600, 2400);
  servoGrip.write(0);
  delay(1000);

  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/move");

  msg.data.data = data_buffer;
  msg.data.size = 4;
  msg.data.capacity = 4;

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &move_callback,
    ON_NEW_DATA);

}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

void pindahKeKoordinat(int targetX, int targetY) {

  long targetStepX = (targetX == 0) ? 0 : (long)(targetX * 16) + 639;
  long targetStepY = (targetY == 0) ? 0 : (long)(targetY * 16) + 639;

  long selisihStepX = targetStepX - currentStepX;
  long selisihStepY = targetStepY - currentStepY;

  digitalWrite(DIR_PIN1, (selisihStepX >= 0) ? HIGH : LOW);
  digitalWrite(DIR_PIN2, (selisihStepY >= 0) ? HIGH : LOW);

  // SUMBU X
  long jalanX = abs(selisihStepX);
  for (long i = 0; i < jalanX; i++) {
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN1, LOW);
    delayMicroseconds(800);
  }

  // SUMBU Y
  long jalanY = abs(selisihStepY);
  for (long i = 0; i < jalanY; i++) {
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(800);
  }

  currentStepX = targetStepX;
  currentStepY = targetStepY;
}