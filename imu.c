#include "imu.h"
#include "i2c.h"
#include "uart.h"
#include <util/delay.h>
#define mpuAddr 0x68
#define accelReg 0x3b
#define gyroReg 0x43

#define alpha 0.3950774372//0.6340686931 // alpha = 1- exp(-2pi*Fc/Fs) Fc = 20 Hz  Fs = 250 Hz
double axf = 0, ayf = 0, azf = 0;

//these are measure with load of 1 g so they will only be applied with significant 
#define scaleFac 16384.0// scale factor for MPU9265 readings to g's

#define zbias 0.3703 // offset in m/s^2 of mpu z axis
#define zscale 0.9932266197 //multiply to normalize the scale of mpu z axis to 1 g

#define ybias -0.11765 // offset in m/s^2 of mpu y axis
#define yscale 0.9977979281 //multiply to normalize the scale of mpu y axis to 1 g

#define xbias 0.2906 // offset in m/s^2 of mpu x axis
#define xscale 0.9981684982 //multiply to normalize the scale of mpu x axis to 1 g

// MPU9250 Gyro sensitivity for ±250 dps range
#define GYRO_SENS 131.0  // LSB/(deg/s)
#define GyroScale 1.0
static float gx_offset=0, gy_offset=0, gz_offset=0;

inline void ema_update(float ax, float ay, float az);

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
    ema_update(axr,ayr,azr);//run filter on new values
    
    *ax = axr;
    *ay = ayr;
    *az = azr;
    
}

void getGyro(float *gx, float *gy, float *gz){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,gyroReg,buf,6);
    
    // Convert raw values to degrees per second
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];
    
    // Convert to degrees per second and apply offset
    *gx = (float)raw_x / GYRO_SENS - gx_offset;
    *gy = (float)raw_y / GYRO_SENS - gy_offset;
    *gz = (float)raw_z / GYRO_SENS - gz_offset;
}

void getGyroRaw(float *gx, float *gy, float *gz){
    uint8_t buf[6];
    i2c_read_bytes(mpuAddr,gyroReg,buf,6);
    
    // Convert raw values to degrees per second
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];
    
    *gx = (float)raw_x / GYRO_SENS*GyroScale;
    *gy = (float)raw_y / GYRO_SENS*GyroScale;
    *gz = (float)raw_z / GYRO_SENS*GyroScale;
}

void gyro_calibrate(){
    int i;
    float x,y,z;
    float x_sum = 0, y_sum = 0, z_sum = 0;

    // Simple calibration - just average the readings
    for(i=0; i<1000; i++){
        getGyroRaw(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        _delay_ms(1);
    }

    // Calculate offsets
    gx_offset = x_sum / 1000.0;
    gy_offset = y_sum / 1000.0;
    gz_offset = z_sum / 1000.0;
    
    // Print calibration values
    
}

//exponential moving average filter
inline void ema_update(float ax, float ay, float az){
    //y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    //y[n] = y[n-1] + alpha*(x[n] - y[n-1])

    axf += alpha*(ax - axf);
    ayf += alpha*(ay - ayf);
    azf += alpha*(az - azf);
}