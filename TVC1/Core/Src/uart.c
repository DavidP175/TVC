#include "uart.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"

//transmits a byte through uart
void uartTransmit(char data){
    CDC_Transmit_FS((uint8_t*)&data, 1);
}

//prints a string through uart
//Use "" not ''
void uartPrint(const char* str){
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

//prints a string with an endline through uart
void uartPrintln(const char* str){
    sprintf(str, "%s\r\n", str);
    uartPrint(str);
}

//prints an int thorugh uart
//use "" not ''
void uartPrint_int(int value){
    char buf[10];
    itoa(value,buf,10);//converts int to ASCII string
    uartPrint(buf);
}

