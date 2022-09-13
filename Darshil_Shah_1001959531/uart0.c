// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"

// PortB masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1
#define UART_DE_MASK 16


//Bit banded address
//#define UART1_EOT (*((volatile uint32_t *)(0x42000000 + (0x4000D030-0x40000000)*32 + 4*4)))
//#define UART1_TXIM (*((volatile uint32_t *)(0x42000000 + (0x4000D038-0x40000000)*32 + 5*4)))
//#define UART1_EOT 16
//#define UART1_TXIM 32
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void initUart0()
{
//    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
//    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

//    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
//    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    _delay_cycles(3);

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= UART_TX_MASK ;                   // enable output on UART0 TX pin
    GPIO_PORTA_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTA_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= UART_TX_MASK | UART_RX_MASK | UART_DE_MASK;    // enable digital on UART0 pins
    GPIO_PORTA_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PA0, PA1
    GPIO_PORTA_PCTL_R &= ~(GPIO_PCTL_PA1_M | GPIO_PCTL_PA0_M); // clear bits 0-7
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                        // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                  // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 ;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
//    UART0_IM_R|=UART_IM_TXIM  ;                                                   // enable TX, RX, and module
//    NVIC_EN0_R |= 1<< (INT_UART0-16);

}



// Initialize UART1
void initUart1()
{
//    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
//    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
 //   SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    _delay_cycles(3);

    // Configure UART1 pins PB0-RX, PB1-TX, PB4 DE
    GPIO_PORTB_DIR_R |= UART_TX_MASK | UART_DE_MASK;                   // enable output on UART0 TX pin
    GPIO_PORTB_DIR_R &= ~UART_RX_MASK;                   // enable input on UART0 RX pin
    GPIO_PORTB_DR2R_R |= UART_TX_MASK | UART_DE_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= UART_TX_MASK | UART_RX_MASK | UART_DE_MASK;    // enable digital on UART0 pins
    GPIO_PORTB_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PB0, PB1
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB1_M | GPIO_PCTL_PB0_M); // clear bits 0-7
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB1_U1TX | GPIO_PCTL_PB0_U1RX;
                                                        // select UART0 to drive pins PB0 and PB1: default, added for clarity

    // Configure UART0 to 34800 baud, 8N1 format
    UART1_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
    UART1_IBRD_R = 65;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 7;                                  // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS ;    // configure for 8N1 w/ 16-level FIFO

//    // Configure DE pin, UART RX andTX
//    GPIO_PORTB_DIR_R |= UART_TX_MASK;                        // make bit 6 an output
//    GPIO_PORTB_DIR_R &= ~UART_RX_MASK;
//    GPIO_PORTB_DR2R_R |= UART_TX_MASK;
//    GPIO_PORTB_DEN_R |= (UART_TX_MASK | UART_RX_MASK );

    //Enabling interrut
    UART1_IM_R|=UART_IM_TXIM | UART_IM_RXIM  ;
    NVIC_EN0_R |= 1<< (INT_UART1-16);
//    UART1_ICR_R |=  UART_ICR_RXIC;

//    UART1_IM_R &=~UART_IM_FEIM

    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN| UART_CTL_EOT ;//| UART_CTL_LBE;        // enable TX, RX, and module
//    UART1_ICR_R|=UART_ICR_RXIC ;
//    UART1_ICR_R|=UART_ICR_TXIC ;

//    UART1_IM_R|=UART_IM_RXIM ;


}


                     // set drive strength to 2mA (not needed since default configuration -- for clarity)

// Configure UART1 pins
//SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;                  // turn-on UART1, leave other uarts in same status
//GPIO_PORTC_DEN_R |= 0x00000070;                           // for DE,Uarttx and Uartrx pins
//GPIO_PORTC_AFSEL_R |= 0x00000030;                         // default, added for clarity
//GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

// Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
/*UART1_CTL_R = 0;                                   // turn-off UART1 to allow safe programming
UART1_CC_R = UART_CC_CS_SYSCLK;                    // use system clock (40 MHz)
UART1_IBRD_R = 65;                                 // r = 40 MHz / (Nx38.4kHz), set floor(r)=65, where N=16
UART1_FBRD_R = 7;                                  // round(fract(r)*64)=6.66
UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN ;  // configure for 8N1 w/ 16-level FIFO
UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
// Set baud rate as function of instruction cycle frequency*/
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128 + 1) >> 1) & 63;   // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint32_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
   while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart0()
{
    return !(UART0_FR_R & UART_FR_RXFE);
}
