#include <math.h>
#include <Wire.h>
#include <vector>
#include "Kalman.h"

using namespace std;

#define MPU_ADDR 0x68
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

class IMU {

    public:
        IMU();
        vector<int16_t> getRawAccel();
        vector<int16_t> getRawGyro();
        vector<int16_t> getRawValues();
        double getPitchRad();
        double getPitchDeg();
        double getGyroYRad();
        double getGyroYDeg();
        vector<double> getPitchAndGyroYRad();
        vector<double> getPitchAndGyroYDeg();
        ~IMU();

    protected:
        // Scale factors for ±2g and ±250 °/s
        constexpr double ACCEL_SCALE = 16384.0; // LSB/g
        constexpr double GYRO_SCALE  = 131.0;   // LSB/(°/s)

        

}