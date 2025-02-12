/*
 * uart_routines2025.c
 *
 *  Created on: Jan 29, 2025
 *      Author: song
 */

#include <uart_routines2025.h>

//UART A0 IO pins on MSP432P4111 chip
// P1.3/UCA0TXD |----> RX PC (echo)
//  P1.2/UCA0RXD |<---- TX PC
#define UARTA0port P1
#define UARTA0pins   BIT2 | BIT3


//UART A1 IO pins
// P2.3/UCA1TXD |----> RX PC (echo)
//  P2.2/UCA1RXD |<---- TX PC
#define UARTA1port P2
#define UARTA1pins   BIT2 | BIT3

//#define SYSTEMCLOCK 12000000    //Hz
//#define BAUDRATE    57600   //bits per seconds (Hz)
//#define ClockPrescalerValue SYSTEMCLOCK/(16*BAUDRATE)
//#define FirstModulationStage 0  //12MHz, 57600 Baud
//#define SecondModulationStage 73 //12MHz, 57600 Baud

//#define SYSTEMCLOCK 12000000    //Hz
//#define BAUDRATE    9600   //bits per seconds (Hz)
//#define ClockPrescalerValue SYSTEMCLOCK/(16*BAUDRATE)
//#define FirstModulationStage 2  //12MHz, 9600 Baud
//#define SecondModulationStage 16 //12MHz, 9600 Baud

//#define SYSTEMCLOCK 48000000    //Hz
//#define BAUDRATE    9600   //bits per seconds (Hz)
//#define ClockPrescalerValue SYSTEMCLOCK/(16*BAUDRATE)
//#define FirstModulationStage 8  //48MHz, 9600 Baud
//#define SecondModulationStage 0xAA //48MHz, 9600 Baud

#define SYSTEMCLOCK 48000000    //Hz
#define BAUDRATE    38400   //bits per seconds (Hz)
#define ClockPrescalerValue SYSTEMCLOCK/(16*BAUDRATE)
#define FirstModulationStage 2  //48MHz, 38400 Baud
#define SecondModulationStage 16 //48MHz, 38400 Baud

//configure UART EUSCI_A0
void ConfigureUART_A0(void) {
    /* Configure UART pins */
    UARTA0port->SEL0 |= UARTA0pins; // set 2-UART pins as secondary function
    UARTA0port->SEL1 &= ~(UARTA0pins);

    /* Configure UART
     *  Asynchronous UART mode, 8O1 (8-bit data, no parity, 1 stop bit),
     *  LSB first by default, SMCLK clock source
     */
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset to configure eUSCI
//bit 15 = 0 to disable parity; bit14=0 Old parity; bit13=0 for LSB first;
//bit12=0 for 8-bit mode; bit11=0 for one stop bit; bits7-6 = 0b10 for SMCLK
//    EUSCI_A0->CTLW0 |= 0b000000010000000;
//even parity
      EUSCI_A0->CTLW0 |= 0b1100000010000000;
    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 57600
     *
     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
     *          values used in next two TODOs
     */
    EUSCI_A0->BRW=ClockPrescalerValue;

    // DONE set clock prescaler in eUSCI_A0 baud rate control register
    EUSCI_A0->MCTLW = (SecondModulationStage<<8)+(FirstModulationStage<<4)+1;    //enalble oversampling
    // DONE configure baud clock modulation in eUSCI_A0 modulation control register


    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt

    NVIC->ISER[0] |= (1)<<EUSCIA0_IRQn;
} //end ConfigureUART_A0(void)


//configure UART EUSCI_A0
void ConfigureUART_A1(void) {
    /* Configure UART pins */
    UARTA1port->SEL0 |= UARTA1pins; // set 2-UART pins as secondary function
    UARTA1port->SEL1 &= ~(UARTA1pins);

    /* Configure UART
     *  Asynchronous UART mode, 8O1 (8-bit data, no parity, 1 stop bit),
     *  LSB first by default, SMCLK clock source
     */
    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset to configure eUSCI
//bit 15 = 0 to disable parity; bit14=0 Old parity; bit13=0 for LSB first;
//bit12=0 for 8-bit mode; bit11=0 for one stop bit; bits7-6 = 0b10 for SMCLK
//    EUSCI_A1->CTLW0 |= 0b0000000010000000;
//even parity
      EUSCI_A1->CTLW0 |= 0b1100000010000000;
    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 57600
     *
     * TODO calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
     *          values used in next two TODOs
     */
    EUSCI_A1->BRW=ClockPrescalerValue;

    // TODO set clock prescaler in eUSCI_A0 baud rate control register
    EUSCI_A1->MCTLW = (SecondModulationStage<<8)+(FirstModulationStage<<4)+1;    //enalble oversampling
    // TODO configure baud clock modulation in eUSCI_A0 modulation control register
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A1->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A1->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt

    NVIC->ISER[0] |= (1)<<EUSCIA1_IRQn;
} //end ConfigureUART_A1(void)

void SendCharArray_A0(char *Buffer) {
    unsigned int count;
    for (count=0; count<strlen(Buffer); count++) {
    // Check if the TX buffer is empty first
      while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
      EUSCI_A0->TXBUF = Buffer[count];
    }   //end for()
} // end SendCharArray(char *Buffer)

//get a ASCII character from UART
//this is a blocking call
char GetChar_A0(void) { //polling
    char ReceivedChar;
//blocking call
//        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG));
//        ReceivedChar=EUSCI_A0->RXBUF;
//        return ReceivedChar;
    if ((EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)==EUSCI_A_IFG_RXIFG)
         ReceivedChar=EUSCI_A0->RXBUF;
    else ReceivedChar=NULL;
        return ReceivedChar;
} //end GetChar(void)

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        // Echo the received character back
        //  Note that reading RX buffer clears the flag and removes value from buffer
        EUSCI_A0->TXBUF = EUSCI_A0->RXBUF;
    }
}

// UART interrupt service routine
void EUSCIA1_IRQHandler(void)
{
    if (EUSCI_A1->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A1->IFG & EUSCI_A_IFG_TXIFG));

        // Echo the received character back
        //  Note that reading RX buffer clears the flag and removes value from buffer
        EUSCI_A1->TXBUF = EUSCI_A1->RXBUF;
    }
}
