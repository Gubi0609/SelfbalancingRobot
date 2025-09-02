#include "IMU.h"

IMU::IMU() {
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

vector<int16_t> IMU::getRawAccel() {

    int16_t ax_raw, ay_raw, az_raw;

    // Read 4 bytes: ACCEL_XOUT_H/L and GYRO_YOUT_H/L
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); // start at accel X high byte
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax_raw = (Wire.read() << 8) | Wire.read();
    ay_raw = (Wire.read() << 8) | Wire.read();
    az_raw = (Wire.read() << 8) | Wire.read();

    return vector<int16_t> = {ax_raw, ay_raw, az_raw};

}

vector<int16_t> IMU::getRawGyro() {

    int16_t gx_raw, gy_raw, gz_raw;

    // Read 4 bytes: ACCEL_XOUT_H/L and GYRO_YOUT_H/L
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); // start at accel X high byte
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    Wire.read(); Wire.read(); // skip accel X
    Wire.read(); Wire.read(); // skip accel Y
    Wire.read(); Wire.read(); // skip accel Z
    Wire.read(); Wire.read(); // skip temp
    gx_raw = (Wire.read() << 8) | Wire.read(); // gyro X
    gy_raw = (Wire.read() << 8) | Wire.read(); // gyro Y
    gz_raw = (Wire.read() << 8) | Wire.read(); // gyro Z

    return vector<int16_t> = {gx_raw, gy_raw, gz_raw};

}

vector<int16_t> IMU::getRawValues() {

    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

    // Read 4 bytes: ACCEL_XOUT_H/L and GYRO_YOUT_H/L
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H); // start at accel X high byte
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax_raw = (Wire.read() << 8) | Wire.read(); // accel X
    ay_raw = (Wire.read() << 8) | Wire.read(); // accel Y
    az_raw = (Wire.read() << 8) | Wire.read(); // accel Z
    Wire.read(); Wire.read(); // skip temp
    gx_raw = (Wire.read() << 8) | Wire.read(); // gyro X
    gy_raw = (Wire.read() << 8) | Wire.read(); // gyro Y
    gz_raw = (Wire.read() << 8) | Wire.read(); // gyro Z

    return vector<int16_t> = {ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw};

}

double IMU::getPitchRad() {

    vector<int16_t> accelRaw = getRawAccel();

    ax_raw = accelRaw(0);
    ay_raw = accelRaw(1);
    az_raw = accelRaw(2);

    // Convert
    double ax_g   = (double)ax_raw / ACCEL_SCALE;      // in g
    double ay_g = (double)ay_raw / ACCEL_SCALE;
    double az_g = (double)az_raw / ACCEL_SCALE;

    double pitch_rad = atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g));

    return pitch_rad;

}

double IMU::getPitchDeg() {

    double pitch_rad = getPitchRad();

    double pitch_deg = pitch_rad * 180.0 / M_PI;

    return pitch_deg;

}

double IMU::getGyroYDeg() {

    vector<int16_t> gyroRaw = getRawGyro();

    gy_raw = gyroRaw(1);

    double gy_dps  = (double)gy_raw / GYRO_SCALE;      // in °/s

    return gy_dps;

}

double IMU::getGyroYRad() {

    double gy_dps = getGyroYDeg();

    double gy_rads = gy_dps * (M_PI / 180.0);          // in rad/s

    return gy_rads;

}

vector<double> IMU::getPitchAndGyroYRad() {

    double pitch_rad = getPitchRad();
    double gy_rads = getGyroYRad();

    return vector<double> = {pitch_rad, gy_rads};

}

vector<double> IMU::getPitchAndGyroYDeg() {

    double pitch_deg = getPitchDeg();
    double gy_dps = getGyroYDeg();

    return vector<double> = {pitch_deg, gy_dps};

}

IMU::~IMU() {
    Wire.endTransmission();
}