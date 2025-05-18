#include <avr/io.h>
#include "I2C.h"
#include "uart.h"
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "imu.h"
#include <math.h>
#include "timer.h"

#define mpuAddr 0x68
#define ACCEL_SCALE 16384.0  // LSB/g for ±2g range scales accel reading to gs
#define gravity 9.81
#define BAUD 57600

// Time step for integration (in seconds)
#define DT 0.01  // 10ms

float tx=0, ty=0, tz=0;

// Function to wrap angle to -180 to 180 degrees
float wrap_angle(float angle) {
    while(angle > 180.0) angle -= 360.0;
    while(angle < -180.0) angle += 360.0;
    return angle;
}

int main(void){

    uartInit(BAUD);
    i2cInit();
    init_timer0();  // Initialize the timer
    gyro_calibrate();
    float gx, gy, gz;
    float tx = 0, ty = 0, tz = 0;
    uint32_t last_time = get_time_ms();  // Initialize with current time
    
    while(1){
        
        
        getGyro(&gx, &gy, &gz);
        
        
        // Calculate time step
        uint32_t current_time = get_time_ms();
        float dt = (current_time - last_time) / 1000.0;  // Convert to seconds
        last_time = current_time;
        
        // Integrate gyro readings
        tx += gx * dt;
        ty += gy * dt;
        tz += gz * dt;
        
        // Wrap angles to -180 to 180 degrees
        tx = wrap_angle(tx);
        ty = wrap_angle(ty);
        tz = wrap_angle(tz);
        char gxStr[10],gyStr[10],gzStr[10],dtStr[10];
        dtostrf(gx, 6, 3, gxStr);
        dtostrf(gy, 6, 3, gyStr);
        dtostrf(gz, 6, 3, gzStr);
        char txStr[10],tyStr[10],tzStr[10];
        dtostrf(tx, 6, 3, txStr);
        dtostrf(ty, 6, 3, tyStr);
        dtostrf(tz, 6, 3, tzStr);
        dtostrf(dt, 6, 3, dtStr);
        
        char str[120];
        sprintf(str,"tx: %s ty: %s tz: %s gx: %s gy: %s gz: %s dt: %s", txStr,tyStr,tzStr, gxStr,gyStr,gzStr,dtStr);
        uartPrintln(str);

    }
    
    return 0;
}
