#include <math.h>
#include "./include/Kalman.h"
#include <Wire.h>

#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

// Scale factors for ±2g and ±250 °/s
constexpr double ACCEL_SCALE = 16384.0; // LSB/g
constexpr double GYRO_SCALE  = 131.0;   // LSB/(°/s)

Matrix<3,1> initialEstimate = {0, 0, 0};
Matrix<3,3> initialCoveriance = {5, 0, 0, 0, 5, 0, 0, 0, 5};
Matrix<2,2> measureError = {0.001, 0, 0, 0.01};
Matrix<3,3> stateError = {0.00001, 0, 0, 0, 0.01, 0, 0, 0, 0.001};
Matrix<2,3> H = {1, 0, 0, 0, 1, 1};
Matrix<3,3> A = {1, 0.05, 0, 0, 1, -0.05, 0, 0, 1};

Kalman gyroAccel = Kalman(initialEstimate, initialCoveriance, measureError, stateError, H, A);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();

  // Accel ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro ±250 °/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  int16_t ax_raw, ay_raw, az_raw, gy_raw;

  // Read 4 bytes: ACCEL_XOUT_H/L and GYRO_YOUT_H/L
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H); // start at accel X high byte
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read(); // skip accel Y
  az_raw = (Wire.read() << 8) | Wire.read(); // skip accel Z
  Wire.read(); Wire.read(); // skip temp
  Wire.read(); Wire.read(); // skip gyro X
  gy_raw = (Wire.read() << 8) | Wire.read(); // gyro Y

  // Convert
  double ax_g   = (double)ax_raw / ACCEL_SCALE;      // in g
  double ay_g = (double)ay_raw / ACCEL_SCALE;
  double az_g = (double)az_raw / ACCEL_SCALE;

  double gy_dps  = (double)gy_raw / GYRO_SCALE;      // in °/s
  double gy_rads = gy_dps * (M_PI / 180.0);          // in rad/s

  double pitch_rad = atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g));
  double pitch_deg = pitch_rad * 180.0 / M_PI;

  Matrix<2,1> z = {pitch_rad, gy_rads};

  Matrix<3,1> result = gyroAccel.estimateDegreesAndRate(z);

  // Output
  Serial.print(pitch_rad);
  Serial.print(" ");
  Serial.print(gy_rads);
  Serial.print(" ");
  Serial.print(result(0));
  Serial.print(" ");
  Serial.println(result(1));

  delay(50);
}

