/*
 * uart_routines2023.h
 *
 *  Created on: Jan 27, 2023
 *      Author: song
 */
#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "csHFXT.h"

void ConfigureUART_A2(void);
void SendCharArray_A2(char *Buffer);
//get a ASCII character from UART
//this is a blocking call
char GetChar_A2(void);


