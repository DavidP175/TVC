#include "main.h"
#include "I2C.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "imu.h"
#include <math.h>
#include "timer.h"

#define mpuAddr 0x68
#define BAUD 115200



float tx=0, ty=0, tz=0;

// Function to wrap angle to -180 to 180 degrees
float wrap_angle(float angle) {
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

int main(void) {
    uartInit(BAUD);
    i2cInit();
    init_timer0();
    gyro_calibrate();
    
    float gx, gy, gz;
    float tx = 0, ty = 0, tz = 0;
    uint32_t last_time = get_time_ms();
    
    while(1) {
        getGyro(&gx, &gy, &gz);
        
        // Calculate time step
        uint32_t current_time = get_time_ms();
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;
        
        // Integrate gyro readings
        tx += gx * dt;
        ty += gy * dt;
        tz += gz * dt;
        
        // Wrap angles to -180 to 180 degrees
        tx = wrap_angle(tx);
        ty = wrap_angle(ty);
        tz = wrap_angle(tz);
        
        char str[120];
        sprintf(str, "tx: %.3f ty: %.3f tz: %.3f gx: %.3f gy: %.3f gz: %.3f dt: %.3f",
                tx, ty, tz, gx, gy, gz, dt);
        uartPrintln(str);
        
        
    }
    
    return 0;
}
