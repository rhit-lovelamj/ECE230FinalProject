#include "msp.h"


/**
 * main.c
 * Authors: Srishti Kamra and Mandolyn Loveland
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
}
