//File Name: ECE230FinalProject.c
/*! \file */
/**********************************************************
 * ECE230 Winter 2024-2025
 * MSP432p4111 Final Project
 * February 20, 2025
 * Description: XXXXXXX
 *
 * Author: Mandolyn Loveland, Srishti Kamra
 *
 *                 MSP432P411x
 *             -------------------      Arduino MEGA
 *         /|\|                   |      ----------
 *          | |                   |     |
 *          --|RST   P3.3/UCA2TXD |---->| P15/RX3
 *            |      P3.2/UCA2RXD |<----| P14/TX3
 *            |                   |     |
 *            |                   |      ----------
 *            |                   |
 *            |                   |
 *            |                   |        LCD
 *            |                   |       --------
 *            |              P2.7 |----->| RS
 *            |                   |      |
 *            |              P2.6 |----->| En
 *            |                   |      |
 *            |                   |  8   |
 *            |               P4  |--\-->| DB
 *            |                   |      |
 *            |                   |       --------
 *            |              PJ.2 |------
 *            |                   |     |
 *            |                   |    HFXT @ 48MHz
 *            |                   |     |
 *            |              PJ.3 |------
 *            |                   |
 *            |                   |
 *            |                   |        LM35
 *            |                   |       --------
 *            |              P5.5 |<-----| Vout
 *            |                   |       --------
 *            |                   |
 *            |                   |
 *            |                   |         MQ3
 *            |                   |       --------
 *            |              P5.4 |<-----| D0
 *            |                   |       --------
 *            |                   |
 *            |                   |
 *            |                  *****************************************************************/
#include "ECE230FinalProject.h"
#include "lcd4bits_ece230w25template.h"  // LCD library, by Dr. Song
#include "uart_routines2025.h"           // UART communication library, by Dr. Song

// Pin Definitions
#define DIGITAL_PORT   P5
#define LM35_PIN       BIT5  // Temperature sensor
#define MQ3_PIN        BIT4  // Alcohol sensor

// Sampling Parameters
#define SAMPLES        10   // Number of samples per second
#define INTERVAL       2    // Display interval (2s)
#define TOTAL_READINGS (SAMPLES * INTERVAL)  // Total readings for averaging

// Buffers for storing sensor data
float temperature_buffer[TOTAL_READINGS];
float hr_buffer[TOTAL_READINGS];
volatile int sample_index = 0;
int hr_samples = 0;

// UART communication buffers
char DataBuffer[50];
char ReceivedChar;
volatile uint32_t timer_overflow_count = 0;

// Function Prototypes
void ADC14_Init(void);
void TimerA_init(void);
float read_LM35(void);
void config_MQ3(void);
float read_MAX30102(void);
void update_averages(void);

/**
 * Initializes ADC14 for temperature sensor (LM35)
 */
void ADC14_init(void) {
    DIGITAL_PORT->SEL0 |= LM35_PIN;
    DIGITAL_PORT->SEL1 |= LM35_PIN;

    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_ON | ADC14_CTL0_SHP;
    ADC14->CTL1 = ADC14_CTL1_RES_2;  // 12-bit resolution
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;  // Channel A5 (P5.5 - LM35)
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

/**
 * Reads temperature from LM35 sensor
 * @return Temperature in Celsius
 */
float read_LM35(void) {
    ADC14->CTL0 |= ADC14_CTL0_SC;  // Start conversion
    while (!(ADC14->IFGR0 & ADC14_IFGR0_IFG0));  // Wait for conversion
    uint16_t adc_value = ADC14->MEM[0];  // Read ADC result
    float voltage = (adc_value * 5) / 4095.0;  // Convert to voltage
    return voltage * 100.0;  // Convert to temperature
}

/**
 * Reads heart rate sensor (MAX30102) value
 * @return Heart rate as a float value
 */
float read_MAX30102(void) {
    do {
        ReceivedChar = GetChar_A2();  // Read incoming data
    } while (ReceivedChar != 'S');
    return (float)(ReceivedChar - '0');
}

/**
 * Configures MQ-3 alcohol sensor digital pin as input
 */
void config_MQ3(void) {
    DIGITAL_PORT->DIR &= ~MQ3_PIN;  // Set P5.4 as input
    DIGITAL_PORT->REN |= MQ3_PIN;   // Enable pull-up/down resistor
    DIGITAL_PORT->OUT &= ~MQ3_PIN;
}

/**
 *Initializes TimerA to generate interrupts every 2 seconds
 */
void TimerA_init(void) {
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_3 | TIMER_A_CTL_CLR;
    TIMER_A0->CCR[0] = (1500000 * 2) - 1;  // 1.5MHz * 2s = 3,750,000 ticks
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;  // Enable TimerA interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);  // Enable TimerA interrupt in NVIC
}

/**
 * \Main function
 */
void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer

    // Configure system clock
    CS->KEY = CS_KEY_VAL;
    CS->CTL0 = CS_CTL0_DCORSEL_3;  // Set DCO to 12MHz
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM_3;
    CS->KEY = 0;

    TimerA_init();
    ConfigureUART_A2();
    ADC14_init();
    lcd4bits_init();
    __enable_irq();

    while (1) {
        temperature_buffer[sample_index] = read_LM35();
        hr_buffer[sample_index] = read_MAX30102();
        hr_samples++;
        sample_index = (sample_index + 1) % TOTAL_READINGS;
        __delay_cycles(12000000 / SAMPLES);
    }
}

/**
 * TimerA ISR, triggers every 2s
 */
void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    timer_overflow_count++;

    if (timer_overflow_count >= 57) {
        timer_overflow_count = 0;
        update_averages();
    }
}

/**
 * Computes and displays averaged temperature and heart rate
 */
void update_averages(void) {
    uint32_t i;
    float temp_sum = 0.0;
    float hr_sum = 0.0;

    for (i = 0; i < TOTAL_READINGS; i++) {
        temp_sum += temperature_buffer[i];
        hr_sum += hr_buffer[i];
    }

    float avg_temp_C = temp_sum / TOTAL_READINGS;
    float avg_temp_F = (avg_temp_C * 9.0 / 5.0) + 32.0;
    float avg_hr = hr_sum / hr_samples;
    hr_samples = 0;

    lcd_SetLineNumber(FirstLine);
    sprintf(DataBuffer, "Temp: %.1f F", avg_temp_F);
    lcd_puts(DataBuffer);

    lcd_SetLineNumber(SecondLine);
    sprintf(DataBuffer, "HR: %.0f BPM", avg_hr);
    lcd_puts(DataBuffer);

    if (DIGITAL_PORT->IN & ~MQ3_PIN) {
        // Turn on LED (Alcohol detected)
    } else {
        // Turn off LED
    }
}
