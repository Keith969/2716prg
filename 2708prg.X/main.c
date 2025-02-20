// ****************************************************************************
//
// File                 : main.c
// Hardware Environment : PIC 16F1789
//                        5v supply voltage
//                        internal oscillator
// Build Environment    : MPLAB IDE
// Version              : V8.76
// By                   : Keith Sabine (keith@peardrop.co.uk)
// Created on           : February 20, 2025, 09:22 AM
//
// ****************************************************************************

#include <pic16f1789.h>
#include "conbits.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "uart.h"

// Useful defines
#define INPUT  0xFF
#define OUTPUT 0x00

// cmds
#define CMD_READ '1'               // Read from the EPROM
#define CMD_WRTE '2'               // Program the EPROM
#define CMD_CHEK '3'               // Check EPROM is blank (all FF))
#define CMD_INIT 'U'               // init the baud rate

// Received chars are put into a queue.
// See e.g. Aho, Hopcroft & Ullman, 'Data structures and Algorithms'
#define QUEUESIZE 1024             // Queue size
#define ENDQUEUE  QUEUESIZE-1      // End of queue
#define HIWATER   QUEUESIZE-16     // The highwater mark, stop sending.

//
// static variables
//
static char    queue[QUEUESIZE];   // The receiver queue
static int16_t head = 0;           // head of the queue
static int16_t tail = ENDQUEUE;    // tail of the  queue
static bool    cmd_active = false; // Are we in a cmd?
static int16_t bytes_pushed = 0;   // pushed into queue
static int16_t bytes_popped = 0;   // popped from queue

// ****************************************************************************
// setCTS()
// Note CTS is active low. So setCTS(1) means 'stop sending'
//
void setCTS(bool b)
{
    PORTAbits.RA2 = b;
}

// ****************************************************************************
// reset the queue
//
void clear()
{
    memset(queue, 0x00, QUEUESIZE);
    head = 0;
    tail = ENDQUEUE; 
    cmd_active   = false;
}

// ****************************************************************************
// Get the next position clockwise in array, handling begin/end of array.
//
int16_t addone(int16_t i)
{
    if (i == ENDQUEUE)
        return 0;
    return i+1;
}

// ****************************************************************************
// How many items are in the queue? Set CTS if more than hiwater mark.
//
int16_t size()
{
    int16_t s = addone(tail) - head;
    if (s > HIWATER) {
        setCTS(true);
    }
    else {
        setCTS(false);
    }
    return s;
}

// ****************************************************************************
// Is the queue empty?
// An empty queue has head one more (clockwise) than tail.
//
bool empty()
{
    if (addone(tail) == head)
        return true;
    return false;
}
// ****************************************************************************
// push a char onto queue.
// to enqueue (push), we move tail one position clockwise.
// e.g. head=0, tail = ENDQUEUE. after push, 
//   queue[0]=c
//   head = 0;
//   tail = 0;
//
void push(char c)
{    
    // If the queue is nearly full, set CTS.)
    int16_t s = addone(tail) - head;
    if (s > HIWATER) {
        setCTS(true);
    }
    else {
        setCTS(false);
    }
        
    if ( addone(addone(tail)) == head) {
        // error - queue is full. Set error led.
        PORTEbits.RE2 = 1;
        __delay_ms(2000);
        return;
    }
    else {
        tail = addone(tail);
        queue[tail] = c;
        bytes_pushed++;
    }  
}

// ****************************************************************************
// pop a char from queue. 
// to dequeue, we move head one position clockwise.
// e.g. after one push, head=0,tail=0
// c = queue[0]
// head = 1;
// tail = 0; (note that now queue is empty as tail+1 == head))
//
char pop()
{
    // pop() is called in write() and could be interrupted, which would
    // cause havoc to the queue. So disable interrupts.)
    INTCONbits.GIE = 0;
    PIE1bits.RCIE=0;
    
    char c = 0;
    
    if (empty()) {
        // error - queue is empty.  Set error led.
        PORTEbits.RE2 = 1;
        __delay_ms(2000);
        return c;
    }
    else {
        c = queue[head];
        head = addone(head);
        bytes_popped++;
    }
    
    // Enable interrupts
    INTCONbits.GIE = 1;
    PIE1bits.RCIE = 1;
    return c;
}

// ****************************************************************************
// first - get the first char pushed on the queue, without removing it.
char first()
{
    return queue[head];
}

// ****************************************************************************
// convert char to hex digit. Handle upper and lower case.
//
uint8_t charToHexDigit(char c)
{
  if (c >= 'a')
    return c - 'a' + 10;
  else if (c >= 'A')
    return c - 'A' + 10;
  else
    return c - '0';
}

// ****************************************************************************
// Initialise the ports
//
void ports_init(void)
{
    // disable analog else weird things happen
    ADCON0bits.ADON = 0;
    ANSELA = 0;           // by default port A is analog
    ANSELB = 0;           // by default port B is analog
    ANSELC = 0;           // by default port C is analog
    ANSELD = 0;           // by default port D is part analog, sigh
    
    // Port A for high address
    // TRISA0 = 0 - A8
    // Also uart control. 
    TRISAbits.TRISA0 = 0; // A8 is output
    TRISAbits.TRISA1 = 0; // A9 is output
    TRISAbits.TRISA2 = 0; // CTS is an active low output
    TRISAbits.TRISA3 = 1; // RTS is an active low input
    TRISAbits.TRISA4 = 0; // unused
    TRISAbits.TRISA5 = 0; // unused
    // bits 6,7 = XTAL
    PORTAbits.RA2 = 0;    // assert CTS

    // Port B for EPROM address A0-A7
    TRISB = OUTPUT;
    
    // Port C is control and uart bits
    // (uart uses bits 6,7). Bits 4/5 spare.
    TRISCbits.TRISC0 = 0; // bit 0 is CE_ chip enable
    TRISCbits.TRISC1 = 0; // bit 1 is WE_ write
    TRISCbits.TRISC2 = 0; // bit 2 is PRG_
    // bit 3,4,5 unused
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC5 = 0;
    // uart bits
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC7 = 1;
    //
    PORTCbits.RC0 = 1; // set CE_ false
    PORTCbits.RC1 = 1; // set WE_ false
    PORTCbits.RC2 = 1; // set PRG_ false
  
    // Port D is data D0-A7, either input or output.
    TRISD = INPUT;
    
    // Port E for status LEDs
    TRISEbits.TRISE0 = 0; // green  LED, while loop
    TRISEbits.TRISE1 = 0; // orange LED, interrupt
    TRISEbits.TRISE2 = 0; // red    LED, warning
    PORTEbits.RE0 = 1;    // and turn them off...
    PORTEbits.RE1 = 0;
    PORTEbits.RE2 = 0;
}

// ****************************************************************************
// high priority service routine for UART receive
//
void __interrupt() isr(void)
{
        char c = 0;

        // Disable interrupts
        INTCONbits.GIE = 0;
        PIE1bits.RCIE=0;

        // Get the character from uart
        bool ok = uart_getc(&c);
        if (ok) {
            // Push the char onto stack
            push(c);

            // Check if we have a cmd yet. 
            int16_t n = size();
            if ( (first() == '$') && n > 1) {
                // We have a command (2 chars at head of queue))
                cmd_active = true;
            }
        }

        // Enable interrupts
        PIE1bits.RCIE=1;
        INTCONbits.GIE = 1;
}

// ****************************************************************************
// Set the address on ports A and B
//
void setup_address(uint16_t addr)
{                
        PORTCbits.RC0 = 1; // set CE_ false
        PORTCbits.RC1 = 1; // set WE_ false (read)
        PORTCbits.RC2 = 1; // set PRG_ false
     
        // Set the address lines. B0-7 is A0-7, A0-1 is A8-9
        uint8_t hi = addr >> 8;
        LATB       = addr & 0x00ff;
        LATA       = hi   & 0x03;
        
        // wait, Tcss
        __delay_us(10);
}

// ****************************************************************************
// Read a byte from port D
//
uint8_t read_port()
{
    // Set port D to input to read from DUT
    TRISD = INPUT;
    
    // wait
    __delay_us(1);
    
    // Set CE_ true, WE_ false
    PORTCbits.RC0 = 0;
    PORTCbits.RC1 = 1;
    __delay_us(1);

    // Read port D
    uint8_t data = PORTD;

    return data;
}

// ****************************************************************************
// Init uart baud rate
//
void do_init()
{
    uint16_t rate;
    char s[8];
        
    rate = uart_init_brg();
    
    sprintf(s, "%d\n", rate);
    uart_puts(s);
}

// ****************************************************************************
// check eprom is wiped clean
// Timing critical code. At 20MHz xtal clock, each instruction = 200nS
//
void do_blank()
{
    uint16_t addr;
    char ads[32];
    bool ok = true;
    char *s;
       
    // Set control bits for reading
    PORTCbits.RC0 = 0; // set CE_ true
    PORTCbits.RC1 = 1; // set WE_ false (read)
    PORTCbits.RC2 = 1; // set PRG_ false
        
    for (addr = 0; addr < 1048; ++addr) {
        if (cmd_active == false) {
            s = "Check aborted\n";
            uart_puts(s);
            return;
        }

        // Latch the 16 bit address.
        setup_address(addr);
        
        // Read port D
        uint8_t data = read_port();
                      
        if (data != 0xff) {
            uart_puts("Erase check fail at address ");
            sprintf(ads, "0x%04x = ", addr);
            uart_puts(ads);
            sprintf(ads, "0x%02x\n", data);
            uart_puts(ads);
            ok = false;
            break;
        }
    }
    
    // Set CE_ false
    PORTCbits.RC0 = 1;
    
    if (ok) {
        s = "Erase check passed.\n";
        uart_puts(s);
    }  
}

// ****************************************************************************
// read from eprom
// Timing critical code. At 20MHz xtal clock, each instruction = 200nS
//
void do_read()
{
    uint16_t addr;
    char ads[16];
    uint8_t col=0;
    
    // Set control bits for reading
    PORTCbits.RC0 = 0; // set CE_ true
    PORTCbits.RC1 = 1; // set WE_ false (read)
    PORTCbits.RC2 = 1; // set PRG_ false
        
    for (addr = 0; addr < 1024; ++addr) {
        if (cmd_active == false) {
            char *s = "Read aborted\n";
            uart_puts(s);
            return;
        }
        
        // Latch the 16 bit address.
        setup_address(addr);
    
        // Read port D
        uint8_t data = read_port();
        
        // Write address
        if (col == 0) {
            sprintf(ads, "%04x: ", addr);
            uart_puts(ads);
        }
        // Write data
        sprintf(ads, "%02x", data);
        uart_puts(ads);
        if (col == 15) {
            col = 0;
            uart_putc('\n');
        } else {
            uart_putc(' ');
            col++;
        }
    }
    
    // set CE_ false
    PORTCbits.RC0 = 1; 
}

// ****************************************************************************
// Write a byte
//
void write_port(uint8_t data)
{
    // Write the byte to port D
     __delay_us(1);
    LATD = data;

    // Activate PGM pulse for 1mS
    __delay_us(10);
    PORTCbits.RC2 = 0; 
    __delay_ms(1);

    // Deactivate PGM pulse
    PORTCbits.RC2 = 1;
    __delay_us(1);
}

// ****************************************************************************
// write to eprom
// Timing critical code. At 20MHz xtal clock, each instruction = 200nS
//
void do_write()
{
    uint16_t addr;
    char ads[48];   
    char c;
    
    // Set port D to output
    TRISD = OUTPUT;
        
    // Set control bits for reading
    PORTCbits.RC0 = 0; // set CE_ true
    PORTCbits.RC1 = 0; // set WE_ true (write)
    PORTCbits.RC2 = 1; // set PRG_ false
      
    for (addr = 0; addr < 1024; addr++) {
        if (cmd_active == false) {
            char *s = "Write aborted\n";
            uart_puts(s);
            return;
        }

        // Get two ascii chars from queue and convert to 8 bit data.
        c = pop();
        uint8_t hi = charToHexDigit(c);
        c = pop();
        uint8_t lo = charToHexDigit(c);
        uint8_t data = hi*16+lo;

        // Latch the 16 bit address.
        setup_address(addr);

        // Write the byte to port D
        write_port(data);
    }
    
    PORTCbits.RC0 = 1; // set CE_ false
    PORTCbits.RC1 = 1; // set WE_ false (read)
    
    // Set port D to input
    TRISD = INPUT;
    
    sprintf(ads, "Write completed.\n");
    uart_puts(ads);
}

// ****************************************************************************
// main
void main(void) {

    // Initialise uart. A value of 0 means use auto baud rate detection.
    uart_init(0);
    
    // Initialise the IO ports
    ports_init();
    
    // Wait for a 'U' char to init the uart BRG
    do_init();
    
    // Enable interrupts
    PIE1bits.RCIE=1;
    INTCONbits.GIE = 1;
        
    // Loop while waiting for commands
    while (true) { 
        if (cmd_active) {
            // Turn on orange LED to show we're active
            PORTEbits.RE1 = 1;
            
            // pop the $
            pop();
            // and the cmd
            char cmd = pop();
            
            // Do the cmd
            if      (cmd == CMD_READ) {
                do_read();
            }
            else if (cmd == CMD_WRTE) {
                do_write();
            }
            else if (cmd == CMD_CHEK) {
                do_blank();
            }
            
            // Clear the cmd
            clear();
        } 
        else {
            // Green LED on to show we're ready
            PORTEbits.RE0 = 1;
        }
        
        // Delay for the loop
        __delay_ms(1);      
    } 
}



