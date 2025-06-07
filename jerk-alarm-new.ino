#include <LSM6DS3.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include "Adafruit_TinyUSB.h"
#define MEASURING_FREQ (1660)

// 正確な時間（秒）で指定
const unsigned long stand_by_time_sec = 30 * 60;   // 30分後に監視開始
const unsigned long running_time_sec =  30 * 60 + 20;  // 30分20秒で終了

static int running_status = 1;
static int accel_flag = 0;
static int accel_limit = 2000;

LSM6DS3 IMU(I2C_MODE, 0x6A);
Madgwick m_;

typedef struct {
  float x;
  float y;
  float z;
} pos3d_t;

pos3d_t gyr_ = { 0 };
pos3d_t acc_ = { 0 };
pos3d_t ang_ = { 0 };
double mes_time_ = 1.0 / (double)MEASURING_FREQ * 1000.0;

void blink(int pin, uint16_t delay_time = 500) {
  digitalWrite(pin, HIGH);
  delay(delay_time);
  digitalWrite(pin, LOW);
  delay(delay_time);
}

void read_gyr() {
  gyr_.x = IMU.readFloatGyroX();
  gyr_.y = IMU.readFloatGyroY();
  gyr_.z = IMU.readFloatGyroZ();
}

void read_acc() {
  acc_.x = IMU.readFloatAccelX();
  acc_.y = IMU.readFloatAccelY();
  acc_.z = IMU.readFloatAccelZ();
}

void setup() {
  pinMode(7, OUTPUT);  // buzzer
  digitalWrite(7, HIGH); delay(500);
  digitalWrite(7, LOW);  delay(500);
  digitalWrite(7, HIGH); delay(500);
  digitalWrite(7, LOW);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.begin(115200);

  IMU.settings.gyroRange = 2000;
  IMU.settings.accelRange = 4;
  while (IMU.begin() != 0) {
    blink(LED_GREEN);
  }

  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x8C);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x8A);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL7_G, 0x00);
  IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x09);
  m_.begin((int)MEASURING_FREQ);
}

void loop() {
  static unsigned long start_time = millis();
  unsigned long elapsed_ms = millis() - start_time;
  float elapsed_sec = elapsed_ms / 1000.0;

  read_acc();
  read_gyr();
  m_.updateIMU(gyr_.x, gyr_.y, gyr_.z, acc_.x, acc_.y, acc_.z);
  ang_.x = m_.getRoll();
  ang_.y = m_.getPitch();
  ang_.z = m_.getYaw();

  delay(mes_time_);

  // jerk検出
  static float last_accel_val = 0;
  float accel_val = ang_.z;
  float accel_diff = (accel_val - last_accel_val) * 100;
  last_accel_val = accel_val;
  int abs_diff = abs(accel_diff);

  static int noaccel_continu_time = 0;
  if (abs_diff < 10) {
    noaccel_continu_time++;
  } else {
    noaccel_continu_time = 0;
  }

  // 監視時間帯か？
  bool in_active_time = (elapsed_sec >= stand_by_time_sec && elapsed_sec <= running_time_sec);

  // 無動作継続中 & 時間内ならブザーON
  if (noaccel_continu_time > accel_limit && in_active_time) {
    accel_flag = 1;
  } else {
    accel_flag = 0;
  }

  if (accel_flag == 1) {
    digitalWrite(7, HIGH);
  } else {
    digitalWrite(7, LOW);
  }

  // debug
  // Serial.println(elapsed_sec);
}
