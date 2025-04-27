#include "uart.h"
#include <util/delay.h>
#include <stdlib.h>


//initializes UART with desired baud rate and
//sets it to 8 data bits, No parity, 1 stop bit
void uartInit(uint32_t baud){
    uint16_t ubrrVal = (F_CPU / (16UL * baud)) - 1;
    //calculates how many clock cycles to wait between
    //bits  based on cpu freq and desired Baud rate
    UBRR0H = (ubrrVal >> 8);//sets the high byte of the baud rate register
    UBRR0L = ubrrVal;//sets the low byte of the baud rate register

    UCSR0B = (1 << TXEN0);
    //enables transmitter (TX) bit in USART Control and Status Register 0 B
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    //sets UCSZ01:0 (UCSZ01 and UCSZ00) to 11 which makes the character size 8 bit
    //leaves UPM01:0 00 to have Parity disabled
    //leaves USBS0 0 to have 1 stop bit
    _delay_ms(100);
}

//transmits a byte through uart
void uartTransmit(char data){
    while(!(UCSR0A & (1 << UDRE0)));//
    //waits until the Data register Empty flag bit
    //in the USART Control and Status reg is 1
    //signifying that the data reg is empty and ready for new data
    UDR0 = data;
    //writes data byte to the USART data reg for transmitting
}

//prints a string through uart
//Use "" not ''
void uartPrint(const char* str){
    while(*str){//iterates through chars in str
        uartTransmit(*str++);
        //transmits char and moves str pointer to next byte/char
    }
}

//prints a string with an endline through uart
void uartPrintln(const char* str){
    uartPrint(str);
    uartTransmit('\r');//transmits carriage return
    uartTransmit('\n');//newline
}

//prints an int thorugh uart
//use "" not ''
void uartPrint_int(int value){
    char buf[10];
    itoa(value,buf,10);//converts int to ASCII string
    uartPrint(buf);
}

