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

void ADC_Init(void);
int Read_Temperature(void);
int cycles;


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
    char DataBuffer[50];
    char ReceivedChar;
    volatile int i;

    while (1) {
        float temp_Sum = 0.0;
        // Read heart rate and SpO2
//        int heart_rate = MAX30102_ReadHeartRate();
//        int spo2 = MAX30102_ReadSpO2();


        // Take 5 readings at 200ms intervals
         for (i = 0; i < SAMPLES; i++) {
             temp_Sum += read_temperature();  // Read and accumulate temperature
             __delay_cycles(12000*200);  // Wait 200ms before next reading
         }

         float temp_F = (((temp_Sum / SAMPLES) * 9.0) / 5.0) + 32.0; //convert to F

        // Format and send data
//        sprintf(buffer, "HR: %d BPM, SpO2: %d%%, Temp: %dC\r\n", heart_rate, spo2, temp);


        sprintf(DataBuffer,"Temp: %f F\r\n", temp_F); //print on COM7 terminal
        SendCharArray_A2(DataBuffer);
        ReceivedChar=GetChar_A2();
        SendCharArray_A2(&ReceivedChar);

        __delay_cycles(12000*2500);  // Wait .5 second before next set of readings

    }
}
