#include <ESP32Servo.h>

// ===== PIN SETUP =====
#define PWMA 25
#define INA1 27
#define INA2 26
#define STBY 13

#define STEP_PIN1 18
#define DIR_PIN1 4
#define STEP_PIN2 19
#define DIR_PIN2 21


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
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, LOW);
  ledcWrite(pwmChannel, 0);
}

void setup() {
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

  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);
}

void loop() {
  servo1.write(0);    // posisi 0 derajat
  delay(2000);

  servo1.write(90);   // posisi 90 derajat
  delay(2000);

  motorKanan(50);   // muter kanan
  delay(1000);

  motorStop();
  delay(1000);

  motorKiri(50);    // muter kiri
  delay(1000);

  motorStop();
  delay(2000);

    for(int i=0;i<22000;i++){
    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(800);

    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(800);
  }
}