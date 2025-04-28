#include <avr/io.h>
#include "I2C.h"
#include "uart.h"
#include <util/delay.h>
#include <stdio.h>
#include "imu.h"
#include <math.h>


#define mpuAddr 0x68
#define ACCEL_SCALE 16384.0  // LSB/g for ±2g range scales accel reading to gs
#define gravity 9.81
#define BAUD 57600

void dtostrf(double val, uint8_t precision, char* str); 

int main(void){
    uartInit(BAUD);//start uart communication
    i2cInit();
    i2c_write_register(mpuAddr,0x6B,0x01);//writes to mpu power mgmnt reg to exit sleep and sets the clock source
    
    while(1){

        float ax,ay,az,azr;
        getAccel(&ax,&ay,&az);
     
        float aMag = sqrt(ax*ax + ay*ay + az*az);
        char axStr[10], ayStr[10], azStr[10], aMagStr[10];
        dtostrf(ax, 3, axStr);
        dtostrf(ay,3, ayStr);
        dtostrf(az,3, azStr);
        dtostrf(aMag,3, aMagStr);

        char str[64];
        sprintf(str, "ax: %s ay: %s az: %s Mag: %s", axStr, ayStr, azStr, aMagStr);
        uartPrintln(str);
        _delay_ms(500);
        
       
    }
    return 0;
}


void dtostrf(double val, uint8_t precision, char *str)
{
    if (precision > 4) precision = 4;

    /* scale up first, then round to nearest integer */
    int32_t scaler = 1;
    uint8_t i;
    for (i = 0; i < precision; i++) scaler *= 10;

    /* convert to fixed-point integer in one shot */
    int32_t fixed = (int32_t) lround(val * scaler);   // needs <math.h>

    /* split into whole and fractional parts */
    int32_t whole = fixed / scaler;
    int32_t frac  = llabs(fixed % scaler);            // always positive

    /* build format string eg “%d.%04d” on the fly */
    char fmt[8];
    sprintf(fmt, "%%d.%%0%dd", precision);

    sprintf(str, fmt, (int)whole, (int)frac);
}