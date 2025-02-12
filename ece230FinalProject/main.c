#include <msp.h>
#include <stdio.h>  //printf(); sprintf();
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <msp432.h>
#include "max30102.h"  // Include library for MAX30102 sensor
#include "uart_routines2025.h"      // UART communication library

#define LM35_PIN    BIT0  // Assuming LM35 is connected to A0 (P1.0)
#define ADC_CHANNEL 0     // Adjusted for MSP432 ADC configuration

void ADC_Init(void);
int Read_Temperature(void);
void UART_Send_String(char *str);

void main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer
    
    // Configure clock (default settings assumed for MSP432)
    CS->KEY = CS_KEY_VAL;
    CS->CTL0 = CS_CTL0_DCORSEL_3;
    CS->KEY = 0;
    
    // Initialize peripherals
    UART_Init();
    MAX30102_Init();
    ADC_Init();
    
    __enable_irq();
    
    char buffer[50];
    while (1) {
        // Read heart rate and SpO2
        int heart_rate = MAX30102_ReadHeartRate();
        int spo2 = MAX30102_ReadSpO2();
        
        // Read temperature
        int temp = Read_Temperature();
        
        // Format and send data
        sprintf(buffer, "HR: %d BPM, SpO2: %d%%, Temp: %dC\r\n", heart_rate, spo2, temp);
        UART_Send_String(buffer);
        
        __delay_cycles(3000000);  // Adjusted delay for MSP432
    }
}

void ADC_Init(void) {
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_ON;
    ADC14->MCTL[0] = ADC_CHANNEL;
    ADC14->CTL1 = ADC14_CTL1_RES_3;
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

int Read_Temperature(void) {
    ADC14->CTL0 |= ADC14_CTL0_SC;  // Start conversion
    while (ADC14->CTL0 & ADC14_CTL0_BUSY);
    int adc_value = ADC14->MEM[0];
    return (adc_value * 330) / 4096;  // Adjusted for 14-bit ADC resolution
}

void UART_Send_String(char *str) {
    while (*str) {
        UART_Send_Char(*str++);
    }
