#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// Function declarations
void getAccel(float *ax, float *ay, float *az);
void getGyro(float *gx, float *gy, float *gz);
void getGyroRaw(float *gx, float *gy, float *gz);
void gyro_calibrate(void);

#endif // IMU_H