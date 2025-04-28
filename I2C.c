#include "I2C.h"
#include <avr/io.h>
#define SCL_CLOCK 100000L
//i2c library for atmega328p


//initiates I2C
//void i2cInit(void)
void i2cInit(void){// call this in the main function before using the i2c methods below
    TWSR = 0x00;//edit TWI status Register. set prescaler bits to 0 so prescaler=1
    TWBR = ((F_CPU/SCL_CLOCK)-16)/2;
    //set bit rate register to control the frequency of scl clock
}
//Starts I2C
//void i2cStart(void)
void i2cStart(void){
    //Edit TWI control Bit to send start condition
    //set TWINT bit to one to set the interrupt flag and tell TWI to start a new operation
    //Set TWSTA bit to one to request a start condtion- tells TWI to pull SDA low while SCL high. falling edge
    //set TWEN bit to high to enable the TWI hardware
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while(!(TWCR & (1<< TWINT)));
    //after setting TWINT, it goes back to zero until start was sent
    //this waits until TWINT is set back to 1 meaning START was sent
}
//Stops I2C
//void i2cStop(void)
void i2cStop(void){
    TWCR = (1<<TWINT) | (1<< TWSTO) | (1<<TWEN);
    //Sets TWSTO to 1 requesting a stop condition.
    // Tells TWI  to set SDA to high while SCL is high. Rising edge
}
//Writes to TWI data register. Takes dev or register address or data that is to be sent
//void i2cWrite(uint8_t data)
void i2cWrite(uint8_t data){
    TWDR = data; //sets TWI data register to address or data byte
    TWCR = (1 << TWINT) | (1 << TWEN); //start transmission
    while(!(TWCR & (1<<TWINT))); //waits for ACK.
}
//sends ACK and returns 1 data byte
//void i2cRead_ack(void)
uint8_t i2cRead_ack(void){
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    //sets TWEA bit to high to enable ACK
    while(!(TWCR & (1<<TWINT))); // wait for incoming byte
    return TWDR;//byte received gets stored in TWI Data register. return that
}
//sends NACK and returns 1 data byte
//uint8_t i2cRead_nack(void)
uint8_t i2cRead_nack(void){
    TWCR = (1 << TWINT) | (1 << TWEN);
    //dont set TWEA bit to high to enable ACK sends NACK
    while(!(TWCR & (1<<TWINT))); // wait for incoming byte
    return TWDR;//byte received gets stored in TWI Data register. return that
}
//initiates i2c connection and write one byte to reg at devAddr
//uint8_t i2c_write_register(uint8_t devAddr, uint8_t reg, uint8_t data)
void i2c_write_register(uint8_t devAddr, uint8_t reg, uint8_t data){
    i2cStart(); //start i2c
    i2cWrite((devAddr<<1)|0);//writes the device address and the mode
    //dev addr is 7 bits so shifts those left and the lsb: 0=write 1=read
    i2cWrite(reg);// send register address
    i2cWrite(data);// send data
    i2cStop();
}
//initiates i2c connection and read one byte from reg at devAddr
//uint8_t i2c_read_register(uint8_t devAddr, uint8_t reg){
uint8_t i2c_read_register(uint8_t devAddr, uint8_t reg){
    i2cStart(); //start i2c
    i2cWrite((devAddr<<1)|0);//writes the device address puts it in write mode to write the reg address
    i2cWrite(reg);// send register address
    i2cStart(); //restarts i2c so we can read
    i2cWrite((devAddr<<1) |1 ); // writes dev address and puts it in read mode
    uint8_t data = i2cRead_nack();// reads byte and sends NACK
    i2cStop();
    return data;
}

//initiates i2c connection and reads (length) bytes from reg at devAddr
//void i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t length){
void i2c_read_bytes(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t length){
    i2cStart(); //start i2c
    i2cWrite((devAddr<<1)|0);//writes the device address puts it in write mode to write the reg address
    i2cWrite(reg);// send register address
    i2cStart(); //restarts i2c so we can read
    i2cWrite((devAddr<<1) |1 ); // writes dev address and puts it in read mode

    uint8_t i;
    for(i=0; i<length-1; i++){//reads all but the last byte with ack 
        buffer[i] = i2cRead_ack();
    }
    buffer[length-1] = i2cRead_nack(); //reads the last byte with nack to stop requesting data

    i2cStop();//stops i2c
}


