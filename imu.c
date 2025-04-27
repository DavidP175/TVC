#include "imu.h"
#include "i2c.h"
#define mpuAddr 0x68
#define accelReg 0x3b
#define gyroReg
#define scaleFac 16384.0// scale factor for MPU9265 readings to g's


void getAccel(float *ax, float *ay, float *az){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,accelReg,buf,6);
    *ax = (float)((buf[0] << 8) | buf[1])/(scaleFac)*9.81;
    *ay = (float)((buf[2] << 8) | buf[3])/(scaleFac)*9.81;
    *az = (float)((buf[4] << 8) | buf[5])/(scaleFac)*9.81;
}

void getGyro(float *gx, float *gy, float *gz){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,gyroReg,buf,6);

    *gx = (float)((buf[0] << 8) | buf[1]);
    *gy = (float)((buf[2] << 8) | buf[3]);
    *gz = (float)((buf[4] << 8) | buf[5]);
}
