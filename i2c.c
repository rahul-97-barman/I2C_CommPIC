/*
 * File:   i2c.c
 * Author: rahul
 *
 * Created on August 23, 2024, 10:47 AM
 */

// PIC18F8770A Configuration Bit Settings
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>

#define _XTAL_FREQ 20000000  // Define crystal frequency (20 MHz)
#define I2C_BaudRate 100000  // Define I2C Baud Rate (100 kHz)

// Function Declarations
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Restart(void);
void I2C_Write(unsigned char data);
unsigned char I2C_Read(unsigned char ack);
void I2C_ErrorHandling(void);
void EEPROM_Write(unsigned char device_addr, unsigned char mem_addr, unsigned char data);
unsigned char EEPROM_Read(unsigned char device_addr, unsigned char mem_addr);

// Main Function
void main(void) {
    unsigned char received_data;

    // Initialize I2C
    I2C_Init();

    while(1) {
        // Write data to EEPROM
        EEPROM_Write(0xA0, 0x00, 0x55); // Device address (0xA0), Memory address (0x00), Data (0x55)
        __delay_ms(10); // Short delay for write operation

        // Read data from EEPROM
        received_data = EEPROM_Read(0xA0, 0x00); // Read from same address
        __delay_ms(500); // Delay before next operation
    }
}

// I2C Initialization Function
void I2C_Init(void) {
    TRISC3 = 1; // Set SCL (RC3) as input
    TRISC4 = 1; // Set SDA (RC4) as input
    SSPCON = 0x28; // Enable I2C Master mode (SSPEN = 1, SSPM3:SSPM0 = 1000)
    SSPADD = (_XTAL_FREQ/(4*I2C_BaudRate))-1; // Set Baud rate
    SSPSTAT = 0x00; // Standard speed mode
}

// I2C Start Condition
void I2C_Start(void) {
    SEN = 1; // Initiate start condition
    while(SEN); // Wait until start condition is complete
}

// I2C Stop Condition
void I2C_Stop(void) {
    PEN = 1; // Initiate stop condition
    while(PEN); // Wait until stop condition is complete
}

// I2C Restart Condition
void I2C_Restart(void) {
    RSEN = 1; // Initiate restart condition
    while(RSEN); // Wait until restart condition is complete
}

// I2C Write Function
void I2C_Write(unsigned char data) {
    SSPBUF = data; // Load data into buffer
    while(BF); // Wait until buffer is full (data sent)
    while((SSPCON2 & 0x1F) || (SSPSTAT & 0x04)); // Wait until transmission is complete
}

// I2C Read Function with ACK/NACK
unsigned char I2C_Read(unsigned char ack) {
    unsigned char received_data;

    RCEN = 1; // Enable receive mode
    while(!BF); // Wait until buffer is full
    received_data = SSPBUF; // Read received data

    if(ack) {
        ACKDT = 0; // ACK after receiving
    } else {
        ACKDT = 1; // NACK after receiving
    }
    ACKEN = 1; // Send acknowledgment
    while(ACKEN); // Wait until ACK/NACK is complete

    return received_data;
}

// I2C Error Handling Function
void I2C_ErrorHandling(void) {
    if(SSPCON & 0x10) { // Check for write collision
        SSPCON &= ~0x10; // Clear write collision flag
    }
    if(BCLIF) { // Check for bus collision
        BCLIF = 0; // Clear bus collision flag
        // Implement additional error recovery (e.g., restart communication)
    }
}

// EEPROM Write Function
void EEPROM_Write(unsigned char device_addr, unsigned char mem_addr, unsigned char data) {
    I2C_Start(); // Start condition
    I2C_Write(device_addr); // Send device address with write flag
    I2C_Write(mem_addr); // Send memory address
    I2C_Write(data); // Send data to write
    I2C_Stop(); // Stop condition
}

// EEPROM Read Function
unsigned char EEPROM_Read(unsigned char device_addr, unsigned char mem_addr) {
    unsigned char data;

    I2C_Start(); // Start condition
    I2C_Write(device_addr); // Send device address with write flag
    I2C_Write(mem_addr); // Send memory address
    I2C_Restart(); // Restart condition
    I2C_Write(device_addr | 1); // Send device address with read flag
    data = I2C_Read(0); // Read data with NACK
    I2C_Stop(); // Stop condition

    return data;
}

