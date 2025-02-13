#include <msp.h>
#include <stdio.h>  //printf(); sprintf();
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <msp432.h>
//#include "max30102.h"  // Include library for MAX30102 sensor
#include "uart_routines2025.h"      // UART communication library

#define LM35_PORT   P5
#define LM35_PIN    BIT5

#define SAMPLES 10  // Number of samples per second
#define INTERVAL 2.5  // Display every 2.5 seconds
#define TOTAL_READINGS (SAMPLES * INTERVAL)  // Total samples to average

volatile float temperature_buffer[TOTAL_READINGS];  // Buffer to store readings
volatile int sample_index = 0;  // Circular index
char DataBuffer[50];
char ReceivedChar;
volatile int i;

void ADC_Init(void);
void TimerA_init(void);
int Read_Temperature(void);


void ADC14_init(void) {
    LM35_PORT->SEL0 |= LM35_PIN;  // Configure P5.5 as ADC input
    LM35_PORT->SEL1 |= LM35_PIN;

    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_ON | ADC14_CTL0_SHP;
    ADC14->CTL1 = ADC14_CTL1_RES_2;  // 12-bit resolution
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_0;  // A0 input
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}


float read_temperature(void) {
    ADC14->CTL0 |= ADC14_CTL0_SC;  // Start conversion
    while (!(ADC14->IFGR0 & ADC14_IFGR0_IFG0));  // Wait for conversion
    uint16_t adc_value = ADC14->MEM[0];  // Read ADC result
    float voltage = (adc_value * 5.0) / 4095.0;  // Convert to voltage
    return voltage * 100.0;  // Convert to temperature
}

// Initialize TimerA to generate interrupt every 2.5s
void TimerA_init(void) {
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_ID_3;  // SMCLK, Up Mode, /8
    TIMER_A0->CCR[0] = (12000000 / 8) * INTERVAL;  // Set count for 2.5s
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;  // Enable interrupt
    NVIC_EnableIRQ(TA0_0_IRQn);  // Enable TimerA interrupt in NVIC
}
void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer

    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    ConfigureUART_A2();
    ADC14_init();  // Initialize ADC14
    __enable_irq();

    while (1) {
        // Read heart rate and SpO2
//        int heart_rate = MAX30102_ReadHeartRate();
//        int spo2 = MAX30102_ReadSpO2();


        // Read temperature and store in circular buffer
        temperature_buffer[sample_index] = read_temperature();
        sample_index = (sample_index + 1) % TOTAL_READINGS;  // Keep index within range
        __delay_cycles(12000000 / SAMPLES);  // Delay for 200ms (12MHz clock)

    }
}
// TimerA ISR (Triggers every 2.5s)
void TA0_0_IRQHandler(void) {
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // Clear interrupt flag

    float temp_sum = 0.0;

    // Compute the average temperature
    for (i = 0; i < TOTAL_READINGS; i++) {
        temp_sum += temperature_buffer[i];
    }

    float avg_temp_C = sum_temperature / TOTAL_READINGS;
    float avg_temp_F = (avg_temperature_C * 9.0 / 5.0) + 32.0;

    // Send the averaged temperature via UART
    sprintf(DataBuffer,"Temp: %f F\r\n", avg_temp_F); //print on COM7 terminal
    SendCharArray_A2(DataBuffer);
    ReceivedChar=GetChar_A2();
    SendCharArray_A2(&ReceivedChar);
}

