// ****************************************************************************
//
// File                 : uart.c
// Hardware Environment : PIC 18F4520
//                        5v supply voltage
//                        internal oscillator
// Build Environment    : MPLAB IDE
// Version              : V8.76
// By                   : Keith Sabine (keith@peardrop.co.uk)
// Created on           : February 20, 2025, 09:22 AM
//
// ****************************************************************************

#include "uart.h"
#include <xc.h>
#include <stdarg.h>
#include <string.h>

// ****************************************************************************
// Function         [ uart_init ]
// Description      [ ]
// ****************************************************************************
void uart_init(const uint32_t baud_rate)
{
    // disable global interrupts
    INTCONbits.GIE = 0;
    
    PIE1bits.TXIE = 1;      // disable TX interrupts
    PIE1bits.RCIE = 1;      // disable RX interrupts
    
    // Set port C UART bits. Both need to be 1
    TRISCbits.TRISC6=1;     // TX
    TRISCbits.TRISC7=1;     // RX
    
    BAUDCONbits.BRG16 = 1;  // 16-bit Baud Rate Generator
    BAUDCONbits.WUE = 0;    // Clear wake up for auto BRG
       
    RCSTAbits.CREN = 1;     // Continuous receive enable
    RCSTAbits.SPEN = 1;     // Serial port enable   
    
    TXSTAbits.SYNC = 0;     // async mode
    TXSTAbits.BRGH = 1;     // baud rate hi
    TXSTAbits.TXEN = 1;     // enable transmit
    
    // If baud_rate is 0, use auto baud detection
    if (baud_rate != 0) {
        // Get the constant for baud rate calculation
        uint8_t factor;
        if (BAUDCONbits.BRG16 && TXSTAbits.BRGH)
            factor = 4;
        else if (BAUDCONbits.BRG16 && !TXSTAbits.BRGH)
            factor = 16;
        else if (!BAUDCONbits.BRG16 && TXSTAbits.BRGH)
            factor = 16;
        else if (!BAUDCONbits.BRG16 && !TXSTAbits.BRGH)
            factor = 64;
        if (TXSTAbits.SYNC)
            factor = 64;

        // Set baud rate
        uint32_t n = (int32_t) _XTAL_FREQ / (factor * baud_rate) - 1u ;
        SPBRGH = (n & 0xFF00) >> 8;
        SPBRG  =  n & 0x00FF;
    }
    
    //RCONbits.IPEN = 1;   // enable priority interrupts
    INTCONbits.PEIE = 1; // enable all unmasked interrupts
    //IPR1bits.RCIP = 1;   // receive interrupts high priority
    
    // Enable UART receive interrupt
    PIE1bits.RCIE=1;  
    // Only set TXIE bit if more data to send, then clear it.
    // The TXIF flag is set whenever TXREG is empty.
    PIE1bits.TXIE=0;  
    
    // enable global interrupts
    INTCONbits.GIE = 1;
}

// ****************************************************************************
// Function         [ uart_init_brg ]
// Description      [ ]
// ****************************************************************************
uint16_t uart_init_brg()
{
    uint16_t rate=0;
    // Disable interrupts
    PIE1bits.RCIE=0;
    
    // Set ABDEN bit
    BAUDCONbits.ABDEN = 1;
        
    // Wait till char received
    while (true) {
        
        // reading RCREG clears RCIF
        if (PIR1bits.RCIF) {
            char c = RCREG & 0x7f; 
            break;
        }
        
        // Flash orange LED while waiting...
        PORTCbits.RC4 = 1;
        __delay_ms(250);
        PORTCbits.RC4 = 0;
        __delay_ms(250);
        
        if ( BAUDCONbits.ABDOVF ) {
            BAUDCONbits.ABDOVF = 0;
        }
    }

    // Return the baudrate from SPBRG
    rate = (SPBRGH << 8) | SPBRG;
    
    // Orange LED off
    PORTCbits.RC4 = 0;
    
    // enable interrupts
    PIE1bits.RCIE=1;
      
    return rate;
}

// ****************************************************************************
// Function         [ uart_getc ]
// Description      [ Receive a char in c. Returns true if OK ]
// ****************************************************************************
bool uart_getc(char *c)
{  
    // NULL if we had an error
    bool ok = false;
    
    // Check for errors
    if (RCSTAbits.FERR) {
        uint8_t er = RCREG;    // Framing error
    }
    else if (RCSTAbits.OERR) {
        RCSTAbits.CREN = 0;    // Overrun error, clear it
        RCSTAbits.CREN = 1;    // by toggling CREN
    }
    else {
        if (PIR1bits.RCIF) {
            *c = RCREG & 0x7f; // strip hi bit
            ok = true;
        }
    } 
    return ok;
}

// ****************************************************************************
// Function         [ uart_putc ]
// Description      [ Send a character ]
// ****************************************************************************
void uart_putc(char c)
{
    // Wait until TXREG ready
    while (PIR1bits.TXIF == 0) {
        NOP();
    }
    
    // Put the char to send in transmit buffer
    TXREG = c;

    // Wait for done
    while(TXSTAbits.TRMT == 0) {
        NOP();
    }
}

// ****************************************************************************
// Function         [ uart_puts ]
// Description      [ Send a null terminated string ]
// ****************************************************************************
void uart_puts(char *s)
{
    // Wait until TXREG ready
    while (PIR1bits.TXIF == 0) {
        NOP();
    }

    // Put the char to send in transmit buffer
    char *p = s;
    while (*p) {
        TXREG = *p++;
        while(TXSTAbits.TRMT == 0) {
            NOP();
        }
    }
}

