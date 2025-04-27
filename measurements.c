#include <avr/io.h>
#include "I2C.h"
#include "uart.h"
#include <util/delay.h>
#include <stdio.h>


#define mpuAddr 0x68
#define ACCEL_SCALE 16384.0  // LSB/g for ±2g range scales accel reading to gs
#define gravity 9.81
#define BAUD 57600

void dtostrf(double val, uint8_t width, uint8_t precision, char* str); 

int main(void){
    uartInit(BAUD);//start uart communication
    i2cInit();
    i2c_write_register(mpuAddr,0x6B,0x01);//writes to mpu power mgmnt reg to exit sleep and sets the clock source
    
    while(1){
        uint8_t buf[6];
        i2c_read_bytes(mpuAddr,0x3b,buf,6);
        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];

        double ax_ms2 = (double) ax/(16384.0)*9.81;
        double ay_ms2 = (double) ay/(16384.0)*9.81;
        double az_ms2 = (double) az/(16384.0)*9.81;
        char axStr[10], ayStr[10], azStr[10];
        dtostrf(ax_ms2, 6, 2, axStr);
        dtostrf(ay_ms2, 6, 2, ayStr);
        dtostrf(az_ms2, 6, 2, azStr);

        char str[64];
        sprintf(str, "ax: %s ay: %s az: %s", axStr, ayStr, azStr);
        uartPrintln(str);
        _delay_ms(500);
    }
    return 0;
}

int32_t mpow(int b, int e){
    int32_t ret = 1;
    uint8_t i;
    for(i=0; i<e; i++) ret *=b;
    return ret;
}


void dtostrf(double val, uint8_t width, uint8_t precision, char* str) {
    int16_t intPart = (int16_t) val;
    int16_t fracPart = (int16_t)((val - intPart) * 100);

    if (fracPart < 0) fracPart = -fracPart;

    // Safe format string, no construction logic
    sprintf(str, "%d.%03d", intPart, fracPart);
}