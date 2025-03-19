// ****************************************************************************
//
// File                 : uart.h
// Hardware Environment : PIC 16F1789
//                        5v supply voltage
//                        internal oscillator
// Build Environment    : MPLAB IDE
// Version              : V8.76
// By                   : Keith Sabine (keith@peardrop.co.uk)
// Created on           : February 20, 2025, 09:22 AM
//
// ****************************************************************************

#ifndef UART_H
#define	UART_H

#include <stdint.h>
#include <stdbool.h>
#include "conbits.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
// Initialise the UART
void uart_init(const uint32_t baud_rate);

// set up the baud rate
uint16_t uart_init_brg();

// Send a char from the UART
void uart_putc(char c);

// Send a string from the UART
void uart_puts(char *s);

// receive a char from the UART
bool  uart_getc(char *c);

#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

