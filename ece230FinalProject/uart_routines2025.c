/*
 * uart_routines2025.c
 *
 *  Created on: Jan 29, 2025
 *      Author: Dr. Song
 */

#include <uart_routines2025.h>

//UART A2 IO pins
// P3.3/UCA2TXD |----> RX PC (echo)
//  P3.2/UCA2RXD |<---- TX PC
#define UARTA2port P3
#define UARTA2pins   BIT2 | BIT3

#define SYSTEMCLOCK 12000000    //Hz
#define BAUDRATE    9600   //bits per seconds (Hz)

//configure UART EUSCI_A2
void ConfigureUART_A2(void)
{
    /* Configure UART pins */
    UARTA2port->SEL0 |= UARTA2pins; // set 2-UART pins as secondary function
    UARTA2port->SEL1 &= ~(UARTA2pins);

    /* Configure UART
     *  Asynchronous UART mode, 8O1 (8-bit data, no parity, 1 stop bit),
     *  LSB first by default, SMCLK clock source
     */
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset to configure eUSCI
//bit 15 = 0 to disable parity; bit14=0 Old parity; bit13=0 for LSB first;
//bit12=0 for 8-bit mode; bit11=0 for one stop bit; bits7-6 = 0b10 for SMCLK
    EUSCI_A2->CTLW0 |= 0b000000010000000;
    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 9600
     *
     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
     *          values used in next two TODOs
     */
    // DONE set clock prescaler in EUSCI_A2 baud rate control register
    EUSCI_A2->BRW = 78;
    // DONE configure baud clock modulation in EUSCI_A2 modulation control register
    EUSCI_A2->MCTLW = 0x0021;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
} //end ConfigureUART_A2(void)

void SendCharArray_A2(char *Buffer)
{
    unsigned int count;
    for (count = 0; count < strlen(Buffer); count++)
    {
        // Check if the TX buffer is empty first
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
            ;
        EUSCI_A2->TXBUF = Buffer[count];
    }   //end for()
} // end SendCharArray(char *Buffer)

//get a ASCII character from UART
//this is a blocking call
char GetChar_A2(void)
{ //polling
    char ReceivedChar;
//blocking call
//        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG));
//        ReceivedChar=EUSCI_A2->RXBUF;
//        return ReceivedChar;
    if ((EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) == EUSCI_A_IFG_RXIFG)
        ReceivedChar = EUSCI_A2->RXBUF;
    else
        ReceivedChar = NULL;
    return ReceivedChar;
} //end GetChar(void)

// UART interrupt service routine
void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG))
            ;

        // Echo the received character back
        //  Note that reading RX buffer clears the flag and removes value from buffer
        EUSCI_A2->TXBUF = EUSCI_A2->RXBUF;
    }
}

