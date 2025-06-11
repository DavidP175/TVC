#include "imu.h"
#include "i2c.h"
#include "uart.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#define mpuAddr 0x68
#define accelReg 0x3b
#define gyroReg 0x43
#define WHO_AM_I 0x75  // Register to check IMU identity
#define PWR_MGMT_1 0x6B // Power management register

#define alpha 0.3950774372//0.6340686931 // alpha = 1- exp(-2pi*Fc/Fs) Fc = 20 Hz  Fs = 250 Hz
float axf = 0, ayf = 0, azf = 0;

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
    
    // Read raw data
    i2c_read_bytes(mpuAddr, gyroReg, buf, 6);
    

    
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
    
    *gx = (float)(raw_x / GYRO_SENS);   
    *gy = (float)(raw_y / GYRO_SENS);
    *gz = (float)(raw_z / GYRO_SENS);
    
}

void gyro_calibrate(){
    int i;
    float x,y,z;
    float x_sum = 0, y_sum = 0, z_sum = 0;

    //wait for imu to stabilize
    HAL_Delay(1000);

    // Simple calibration - just average the readings
    for(i=0; i<2000; i++){
        getGyroRaw(&x, &y, &z);
        x_sum += x;
        y_sum += y;
        z_sum += z;
        HAL_Delay(2);
    }

    // Calculate offsets
    gx_offset = x_sum / 2000.0;
    gy_offset = y_sum / 2000.0;
    gz_offset = z_sum / 2000.0;
    
    
}

//exponential moving average filter
inline void ema_update(float ax, float ay, float az){
    //y[n] = alpha*x[n] + (1-alpha)*y[n-1]
    //y[n] = y[n-1] + alpha*(x[n] - y[n-1])

    axf += alpha*(ax - axf);
    ayf += alpha*(ay - ayf);
    azf += alpha*(az - azf);
}

// Add this new function to initialize the IMU
void imu_init(void) {
    
    /*
    uint8_t who_am_i;
    char debug_str[64];
    // Read WHO_AM_I register
    who_am_i = i2c_read_register(mpuAddr, WHO_AM_I);
    sprintf(debug_str, "WHO_AM_I: 0x%02X", who_am_i);
    uartPrintln(debug_str);
    
    if (who_am_i != 0x68) {  // MPU6050 should return 0x68
        uartPrintln("ERROR: IMU not responding correctly!");
        return;
    }
    */
    // Wake up the IMU
    i2c_write_register(mpuAddr, PWR_MGMT_1, 0x00);
    HAL_Delay(10);
    
    // Configure gyro range to ±250°/s
    i2c_write_register(mpuAddr, 0x1B, 0x00);
    HAL_Delay(10);
    
    // Configure accelerometer range to ±2g
    i2c_write_register(mpuAddr, 0x1C, 0x00);
    HAL_Delay(10);
    
    /*
    // Read back configuration to verify
    uint8_t pwr_mgmt = i2c_read_register(mpuAddr, PWR_MGMT_1);
    uint8_t gyro_config = i2c_read_register(mpuAddr, 0x1B);
    uint8_t accel_config = i2c_read_register(mpuAddr, 0x1C);
    
    sprintf(debug_str, "PWR_MGMT_1: 0x%02X, GYRO_CONFIG: 0x%02X, ACCEL_CONFIG: 0x%02X",
            pwr_mgmt, gyro_config, accel_config);
    uartPrintln(debug_str);
    */
    gyro_calibrate();
}