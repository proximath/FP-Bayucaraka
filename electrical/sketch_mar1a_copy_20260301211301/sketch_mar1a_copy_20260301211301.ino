#include <ESP32Servo.h>

#define STEP_PIN1 25
#define DIR_PIN1 26
#define STEP_PIN2 32
#define DIR_PIN2 33

long currentStepX = 0;
long currentStepY = 0;

Servo servoLift;
Servo servoGrip;

int servoPinLift = 23;
int servoPinGrip = 22;
int pos = 0;

void setup() {
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  Serial.begin(115200);
  Serial.println("Sistem Siap. Masukkan: PayX PayY DropX DropY");

  servoLift.attach(servoPinLift, 600, 2400);
  servoGrip.attach(servoPinGrip, 600, 2400);
  
  servoLift.write(0);
  servoGrip.write(0);
  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    int payX = Serial.parseInt();
    int payY = Serial.parseInt();
    int dropX = Serial.parseInt();
    int dropY = Serial.parseInt();

    Serial.println("================================");
    
    //TITIK PAYLOAD
    Serial.println("Step 1: Menuju Payload...");
    pindahKeKoordinat(payX, payY);
    
//GRIPPER TURUN
  for (pos = 0; pos <= 180; pos += 1) {
    servoLift.write(pos);
    delay(20);
  }
  delay(500);

  //TUTUP GRIP 
  for (pos = 0; pos <= 85; pos += 1) {
    servoGrip.write(pos);
    delay(20);
  }
  delay(500); 
  servoGrip.detach();

  //GRIPPER NAIK 
  for (pos = 180; pos >= 0; pos -= 1) {
    servoLift.write(pos);
    delay(20);
  }
  delay(2000);

    //TITIK DROPPING
    Serial.println("Step 2: Menuju Dropping...");
    pindahKeKoordinat(dropX, dropY);
    delay(2000); 

  //BUKA GRIP
servoGrip.attach(servoPinGrip, 600, 2400);
for (pos = 85; pos >= 0; pos -= 1) {
  servoGrip.write(pos);
  delay(20);
}
delay(500);

    //KEMBALI KE TITIK AWAL (0,0)
    Serial.println("Step 3: Kembali ke Titik Awal (0,0)...");
    pindahKeKoordinat(0, 0);

    Serial.println("Siklus Selesai.");
    Serial.println("================================");
  }
}

void pindahKeKoordinat(int targetX, int targetY) {
  
  long targetStepX = (targetX == 0) ? 0 : (long)(targetX * 16) + 639;
  long targetStepY = (targetY == 0) ? 0 : (long)(targetY * 16) + 639;

  
  long selisihStepX = targetStepX - currentStepX;
  long selisihStepY = targetStepY - currentStepY;

  
  digitalWrite(DIR_PIN1, (selisihStepX >= 0) ? HIGH : LOW);
  digitalWrite(DIR_PIN2, (selisihStepY >= 0) ? HIGH : LOW);

  //SUMBU X
  long jalanX = abs(selisihStepX);
  for (long i = 0; i < jalanX; i++) {
    digitalWrite(STEP_PIN1, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN1, LOW);
    delayMicroseconds(800);
  }

  //SUMBU Y
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
