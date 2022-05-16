#ifndef _MSP_CLOCK_H_
#define _MSP_CLOCK_H_
#include <msp430.h>
#include <stdint.h>

#define CLOCK_SMCLK_1MHZ
//#define CLOCK_SMCLK_8MHZ

void mspClockSet();
void delay(int32_t msec_c);
void delayMicroseconds(uint16_t us);

#endif
