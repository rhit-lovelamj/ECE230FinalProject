#include <msp.h>
#include <stdio.h>  //printf(); sprintf();
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <msp432.h>
//#include "max30102.h"  // Include library for MAX30102 sensor
#include "uart_routines2025.h"      // UART communication library

#define LM35_PORT   P5
#define LM35_PIN    BIT5  //


void ADC_Init(void);
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
void main(void) {
    volatile uint32_t i;
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // Stop watchdog timer

//    ConfigureUART_A0();
    ADC14_init();  // Initialize ADC14
    __enable_irq();

    while (1) {
        // Read heart rate and SpO2
//        int heart_rate = MAX30102_ReadHeartRate();
//        int spo2 = MAX30102_ReadSpO2();

        // Read temperature
        float temp_C = read_temperature();
        float temp_F = (temp_C * 9.0 / 5.0) + 32.0; //convert to F

        // Format and send data
//        sprintf(buffer, "HR: %d BPM, SpO2: %d%%, Temp: %dC\r\n", heart_rate, spo2, temp);
        printf("Temp: %f F\r\n", temp_F); //print on Console

        for (i = 100000; i > 0; i--)
            ;        // lazy delay
    }
}
