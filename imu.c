#include "imu.h"
#include "i2c.h"
#define mpuAddr 0x68
#define accelReg 0x3b
#define gyroReg

//these are measure with load of 1 g so they will only be applied with significant 
#define scaleFac 16384.0// scale factor for MPU9265 readings to g's

#define zbias 0.3703 // offset in m/s^2 of mpu z axis
#define zscale 0.9932266197 //multiply to normalize the scale of mpu z axis to 1 g

#define ybias -0.11765 // offset in m/s^2 of mpu y axis
#define yscale 0.9977979281 //multiply to normalize the scale of mpu y axis to 1 g

#define xbias 0.2906 // offset in m/s^2 of mpu x axis
#define xscale 0.9981684982 //multiply to normalize the scale of mpu x axis to 1 g

//TODO filter accel noise before sensor fusion?
void getAccel(float *ax, float *ay, float *az){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,accelReg,buf,6);
    float axr, ayr, azr;
    axr = (float)((buf[0] << 8) | buf[1])/(scaleFac)*9.81;
    ayr = (float)((buf[2] << 8) | buf[3])/(scaleFac)*9.81;
    azr = (float)((buf[4] << 8) | buf[5])/(scaleFac)*9.81;

    
    if(axr>5.0){
        axr = (axr - xbias)*xscale;
    }
    else{
        axr -= 0.15;
    }
    if(ayr>5.0){
        ayr = (ayr - ybias)*yscale;
    }
    else{
        ayr -= 0.01;
    }
    if(azr>5.0){
        azr = (azr - zbias)*zscale;
    }
    else{
        azr -= 0.15;
    }
    
    *ax = axr;
    *ay = ayr;
    *az = azr;
}
/*
void getGyro(float *gx, float *gy, float *gz){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,gyroReg,buf,6);

    *gx = (float)((buf[0] << 8) | buf[1]);
    *gy = (float)((buf[2] << 8) | buf[3]);
    *gz = (float)((buf[4] << 8) | buf[5]);
}
*/