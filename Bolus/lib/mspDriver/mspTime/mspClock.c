#include "mspClock.h"

void mspClockSet()
{
#if defined(CLOCK_SMCLK_1MHZ)
#if defined(__MSP430G2553__)
	BCSCTL1 = CALBC1_1MHZ; // Set DCO Clock to 1MHz
	DCOCTL = CALDCO_1MHZ;
#elif defined (__MSP430FR2433__)
#endif
#elif defined(CLOCK_SMCLK_8MHZ)
#error "Clock Selected not in range"
#else
#error "NO Clock Selected"
#endif
}

void delay(int32_t msec_c)
{
	do
	{
#if defined(CLOCK_SMCLK_1MHZ)
		__delay_cycles(1000);
#elif defined(CLOCK_SMCLK_8MHZ)
		__delay_cycles(8000);
#endif
		msec_c--;
	} while (msec_c);
}

void delayMicroseconds(uint16_t us)
{
	do
	{
#if defined(CLOCK_SMCLK_1MHZ)
		__delay_cycles(1);
#elif defined(CLOCK_SMCLK_8MHZ)
		__delay_cycles(8);
#endif
	} while (--us);
}
