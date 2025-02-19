#include <msp.h>
#include <stdio.h>  //printf(); sprintf();
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <msp432.h>
#include "lcd4bits_ece230w25template.h"  // Include library for LCD 4Bits
#include "uart_routines2025.h"      // UART communication library

#define DIGITAL_PORT   P5
#define LM35_PIN    BIT5
#define MQ3_PIN     BIT4

#define SAMPLES 10  // Number of samples per second
#define INTERVAL 2  // Display every 2.5 seconds
#define TOTAL_READINGS (SAMPLES * INTERVAL)  // Total samples to average

float temperature_buffer[20];  // Buffer to store readings
volatile int sample_index = 0;  // Circular index
char DataBuffer[50];
char ReceivedChar;
uint32_t i;
volatile uint32_t timer_overflow_count = 0;

void ADC14_Init(void);
void TimerA_init(void);
float read_LM35(void);
void config_MQ3(void);
void update_averages(void);

void ADC14_init(void)
{
    DIGITAL_PORT->SEL0 |= LM35_PIN;
    DIGITAL_PORT->SEL1 |= LM35_PIN;

    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_ON | ADC14_CTL0_SHP;
    ADC14->CTL1 = ADC14_CTL1_RES_2;  // 12-bit resolution

    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;  // Channel A5 (P5.5 - LM35)
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

// Function to convert LM35 ADC value to temperature in Celsius
float read_LM35(void)
{
    ADC14->CTL0 |= ADC14_CTL0_SC;  // Start conversion
    while (!(ADC14->IFGR0 & ADC14_IFGR0_IFG0))
        ;  // Wait for conversion
    uint16_t adc_value = ADC14->MEM[0];  // Read ADC result
    float voltage = (adc_value * 3.3) / 4095.0;  // Convert to voltage
    return voltage * 100.0;  // Convert to temperature
}

// Function to convert MQ-3 ADC value to alcohol concentration (arbitrary unit)
void config_MQ3(void)
{
    // Set P5.4 as input for digital read from MQ-3 DOUT
    DIGITAL_PORT->DIR &= ~MQ3_PIN;  // Set P5.1 as input
    DIGITAL_PORT->REN |= MQ3_PIN;   // Enable pull-up/down resistor
    DIGITAL_PORT->OUT &= ~MQ3_PIN;   // Enable pull-up/down resistor

}

// Initialize TimerA to generate interrupt every 2s
void TimerA_init(void)
{
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 |  // SMCLK (12MHz)
            TIMER_A_CTL_MC_1 |      // Up Mode (counts to CCR0)
            TIMER_A_CTL_ID_3 |      // Divide by 8 (12MHz / 8 = 1.5MHz)
            TIMER_A_CTL_CLR;        // Clear timer

    TIMER_A0->CCR[0] = (1500000 * 2) - 1;  // 1.5MHz * 2s = 3,750,000 ticks
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;  // Enable TimerA interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);  // Enable TimerA interrupt in NVIC
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer

    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3; // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                      // Lock CS module from unintended accesses

    TimerA_init();
    ConfigureUART_A2();
    ADC14_init();
    lcd4bits_init();
    __enable_irq();

    // Read temperature and store in circular buffer
    while (1)
    {
        temperature_buffer[sample_index] = read_LM35();
        sample_index = (sample_index + 1) % TOTAL_READINGS;

        __delay_cycles(12000000 / SAMPLES);
    }


}
// TimerA ISR (Triggers every 2.5s)
void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // Clear interrupt flag
    timer_overflow_count++;

    if (timer_overflow_count >= 57)  // Every 2.5 seconds
    {
        timer_overflow_count = 0;

        // Update averages
        update_averages();
    }
}

// Function to calculate and send average data
void update_averages(void)
{
    float temp_sum = 0.0;

    // Calculate the sum of values
    for (i = 0; i < TOTAL_READINGS; i++)
    {
        temp_sum += temperature_buffer[i];

    }

    // Calculate averages
    float avg_temp_C = temp_sum / TOTAL_READINGS;
    float avg_temp_F = (avg_temp_C * 9.0 / 5.0) + 32.0;


    // Send the averaged data via UART
    lcd_SetLineNumber(FirstLine);
    sprintf(DataBuffer, "Avg Temp: %.2f F \r\n", avg_temp_F);
    lcd_puts(DataBuffer);
//    lcd_SetLineNumber(SecondLine);
//    lcd_puts(DataBuffer);

    // Alcohol detection
    if (DIGITAL_PORT->IN &~ MQ3_PIN)
    {
        sprintf(DataBuffer, "Alcohol Detected!\r\n");
    }
    else
    {
        sprintf(DataBuffer, "Alcohol Not Detected\r\n");
    }
    SendCharArray_A2(DataBuffer);
}


