#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rclc/node.h>
#include <rclc/publisher.h>
#include <rclc/subscription.h>
#include <rclc/client.h>
#include <rclc/service.h>
#include <geometry_msgs/msg/point.h>
#include <std_srvs/srv/empty.h>
#include <rcl/allocator.h>
#include <rclc/executor.h>
#include <ESP32Servo.h>

#define PWMA 25
#define INA1 27
#define INA2 26
#define STBY 13

#define STEP_PIN1 18
#define DIR_PIN1 4
#define STEP_PIN2 19
#define DIR_PIN2 21

rcl_allocator_t allocator;
rclc_support_t support;
rcl_subscription_t move_subscriber;

rcl_node_t node;
rclc_executor_t executor;


// PWM Setup
const int pwmChannel = 0;
const int pwmFreq = 200;
const int pwmResolution = 8; // 0-255

Servo servo1;
int pinServo = 22;   // GPIO22 (D22)

void motorKanan(int speed) {
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, LOW);
  ledcWrite(pwmChannel, speed);
}

void motorKiri(int speed) {
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
  ledcWrite(pwmChannel, speed);
}

void motorStop() {
  digitalWrite(INA1, HIGH);
  digitalWrite(INA2, HIGH);
  ledcWrite(pwmChannel, 0);
}

void move_callback(const void*);
geometry_msgs__msg__Point move_position;

void setup() {
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_board", "", &support);

  rclc_subscription_init_default(
    &move_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "move"
  );

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &move_subscriber, &move_position, move_callback, ON_NEW_DATA);

  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // Aktifkan driver TB6612

  // PWM setup ESP32
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(PWMA, pwmChannel);
  servo1.attach(pinServo);   // hubungkan servo ke GPIO22

  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  //digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);
}

int lastx = 0, lasty = 0;

void grab(){
  motorKanan(50);   // muter kanan
  delay(2000);

  motorStop();
  delay(1000); 

  servo1.write(0);    // posisi 0 derajat
  delay(2000);

  motorKiri(50);    // muter kiri
  delay(2000);

  motorStop();
  delay(2000);
}

void drop(){
  motorKanan(50);   // muter kanan
  delay(2000);

  motorStop();
  delay(1000); 

  servo1.write(90);    // posisi 0 derajat
  delay(2000);

  motorKiri(50);    // muter kiri
  delay(2000);

  motorStop();
  delay(2000);

  move(0, 0);
}

int action_idx = 0;

void move(int x, int y){
  int dirx = (x > lastx);
  int diry = (y > lasty);
  int stepx = abs(x - lastx) * 1375 / 39;
  int stepy = abs(y - lasty) * 1471 / 41;

  digitalWrite(DIR_PIN1, dirx);
  digitalWrite(DIR_PIN2, diry);
  
  for(int i = 0; i < stepx; i++){
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN1, LOW);
    delayMicroseconds(800);
  }

  for(int i = 0; i < stepy; i++){
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(5);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(800);
  }

  if (action_idx == 0){
    grab();
    action_idx++;
  } else if (action_idx == 1) {
    drop();
    action_idx++;
  } else {
    action_idx++;
  }
  action_idx = action_idx % 3;
  lastx = x;
  lasty = y;
}

void move_callback(const void *message){
  const geometry_msgs__msg__Point *msg = (geometry_msgs__msg__Point*)message;
  int x = msg->x;
  int y = msg->y;
  move(x, y);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}