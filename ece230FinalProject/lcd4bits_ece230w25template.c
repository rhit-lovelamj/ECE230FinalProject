//File Name: lcd4bits_ece230w25template.c
//Author: Jianjian Song
//Date: January 29, 2025
//ECE230 Winter 2024-2025
//
/*
 *	LCD Display Panel Driver Routines for 4-bit interface.
 *	Only Pins 11-14 for Bits 4-7 are used.
 *  Reference:  Refer to the Optrex LCD User Manual for all details and command formats
 *              used in these routines "Optrex LCD Manual Dmcman_full-user manual.pdf" 
 *
 *	All necessary LCD and delay functions have been bundled into this file,
 *  and the necessary function prototype statements are bundled into the 
 *  companion header file, lcd4bit.h.  Therefore by including lcd4bit.c into 
 *  a project along with a separate "main.c" program that calls one or more of these
 *  the LCD panel display routines and contains an "#include "lcd8bit.h" header file,
 *  you need not cut and paste all of these routines into the main program file itself.
 *
 *	This code will interface to a wide variety of B/W LCD Display Controllers
 *	like the Hitachi HD44780. It uses the 8-bit transfer mode, and it assumes that the
 *  pins of PORT D (RD7:0) are connected to the LCD panel, with
 *	the hardware connected as follows, assuming the standard 16 pin LCD pinout:
 
 *    Pin 1: Vss = GND
 *    Pin 2: VDD = +5V
 *    Pin3: Contrast adjustment, 5V to maximum contrast
 *    Pin 4: Register Select RS, RS=1 for command and RS=0 for data
 *    Pin 5: R/W* to be connected to GND for write only
 *    Pin 6: E or clock pin, falling edge to latch data line
 *	  Pins 7-14: LCD data bits 0-7 LCD
 *	  Pin 15: +4.2V for LED back light
 *	  Pin 16: GND for LED back light
 *	
 *	The available routines in this file are:
 *
 *    1.  lcd4bits_init( )
 *        Always call lcd_init() first, which follows the manufacturer's
 *        directions for initializing the LCD display panel into 8-bit transfer mode.
 *        Then you may call any of the other routines, as needed.  Note that this
 *        initialization routine also makes RD7:0 all outputs, as required to drive
 *		  the LCD panel connected to RD7:0.  It also selects 8 MHz internal clock.
 *
 *    2.  lcd_clear( )
 *        Clears LCD display and homes the cursor
 *
 *	  3.  lcd_puts(const char s*)
 *        writes a constant character string to the lcd panel
 *
 *    4.  lcd_putch(char s)
 *        writes a single character to lcd panel
 *
 *    5.  lcd_goto(unsigned char pos)
 *        Moves cursor to desired position.  For 16 x 2 LCD display panel, 
 *        the columns of Row 1 are 0x00....0x10
 *        the columns of Row 2 are 0x40....0x50
 *
 *	  6.  DelayMs(unsigned int NrMs)
 *		  Delays for NrMs milliseconds.
 *
 */
#include <lcd4bits_ece230w25template.h>
#include <msp.h>

void DelayMs(unsigned int);

/*
 * Initialize the LCD - put into 8 bit mode
 */
/*                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST          P2.7 |<--Register Select RS
 *            |             P2.6 |<--E Clock
 *            |                  |
 *            |          Port 4  |--> Data Lines (bits 4-7)
 *            |                  |
 *            |                  |
 */
//define Ports and Pins to interface LCD
#define LCD_CONTROL_PORT P2 //control signal port
#define LCD_RS          7        //register select pin -LCD command/data
#define LCD_EN          6         //E clock pin -LCD Enable, falling edge active
#define LCD_DATA        P4       //Data port -LCD Data pins (D0 - D7)
#define LCD_DATA_PINS       (BIT4 | BIT5 | BIT6 | BIT7)

#define Set_Command_Mode LCD_CONTROL_PORT->OUT = (LCD_CONTROL_PORT->OUT) & (~(0b1<<LCD_RS));
#define Set_Data_Mode LCD_CONTROL_PORT->OUT = (LCD_CONTROL_PORT->OUT) | (0b1<<LCD_RS);

#define Set_Enable_Low  LCD_CONTROL_PORT->OUT =  (LCD_CONTROL_PORT->OUT) & ~(0b1<<LCD_EN);
#define Set_Enable_High  LCD_CONTROL_PORT->OUT =  (LCD_CONTROL_PORT->OUT) | (0b1<<LCD_EN);

void lcd4bits_init(void)
{
    LCD_CONTROL_PORT->DIR |= (1 << LCD_RS | 1 << LCD_EN);
            LCD_CONTROL_PORT->SEL0 &= ~(1 << LCD_RS | 1 << LCD_EN);
            LCD_CONTROL_PORT->SEL1 &= ~(1 << LCD_RS | 1 << LCD_EN);
            LCD_DATA->DIR |= LCD_DATA_PINS;
            LCD_DATA->SEL0 &= ~LCD_DATA_PINS;
            LCD_DATA->SEL1 &= ~LCD_DATA_PINS;

            Set_Command_Mode;
            Set_Enable_Low;

            lcd4bits_write(CMD_MODE, 0x33);
            DelayMs(10);
            lcd4bits_write(CMD_MODE, 0x32);
            DelayMs(20);
            //lcd4bit_write_nibble(CMD_MODE, 0x28);
            lcd4bits_write(CMD_MODE, 0x28);
            lcd4bits_write(CMD_MODE, LCDCMD_DisplaySettings);
            lcd_clear();
            lcd4bits_write(CMD_MODE, LCDCMD_EMS);
} //end lcd8bits_init()

//making E line rise and then fall.  This falling edge
//writes data on LCD Panel pin DB7-0 into LCD Panel.
void LCD_STROBE(void) {
    Set_Enable_High
    DelayMs(10);
    Set_Enable_Low
}

/*
 * lcd_write function ---writes a byte to the LCD in 8-bit mode
 * Note that the "mode" argument is set to either CMD_MODE (=0) or DTA_MODE (=1), so that the
 * LCD panel knows whether an instruction byte is being written to it or an ASCII code is being written to it
 * that is to be displayed on the panel.
 */ 
void lcd8bits_write(unsigned char mode, unsigned char CmdChar) {

    if(mode==CMD_MODE) Set_Command_Mode //LCD_CONTROL_PORT->OUT = (LCD_CONTROL_PORT->OUT) & (~(0b1<<LCD_RS));
    else {Set_Data_Mode; }
    DelayMs(10);
    LCD_DATA->OUT = CmdChar;
    LCD_STROBE(); // Write 8 bits of data on D7-0
}


void lcd4bits_write(unsigned char mode, unsigned char CmdChar) {
    if(mode == CMD_MODE){
        Set_Command_Mode
    }
    else{
        Set_Data_Mode;
    }
    DelayMs(10);
    LCD_DATA->OUT = (LCD_DATA->OUT & 0x0F) | (CmdChar & 0xF0);
    LCD_STROBE();
    LCD_DATA->OUT = (LCD_DATA->OUT & 0x0F) | (CmdChar << 4 & 0xF0);
    LCD_STROBE();
    DelayMs(10);
}


void lcd_puts(char *string) {
    while (*string != 0) // Last character in a C-language string is alway "0" (ASCII NULL character)
        lcd4bits_write(DATA_MODE, *string++);
}

/*
 * 	Clear and home the LCD
 */

void lcd_clear(void) {
    lcd4bits_write(CMD_MODE, LCDCMD_ClearDisplay);
    DelayMs(2);
}

void lcd_putch(char character) {
    lcd4bits_write(DATA_MODE, character);
}

/*
 * Moves cursor to desired position.  
 * For 16 x 2 LCD display panel, 
 *     the columns of Row 1 are 0x00....0x10
 *     the columns of Row 2 are 0x40....0x50
 */

void lcd_SetLineNumber(unsigned char position) {
    lcd4bits_write(CMD_MODE, 0x80 | position); // The "cursor move" command is indicated by MSB=1 (0x80)
    // followed by the panel position address (0x00- 0x7F)
}


void DelayMs(unsigned int nrms) {
    unsigned int i, j;
    for (j = 0; j < nrms; j++)
        for (i = 0; i < 20; i++);
}


