/*
 * mspWDT.c
 *
 *  Created on: Nov 24, 2020
 *      Author: Asus
 */

#include "mspWDT.h"
#include <msp430.h>

volatile func_t _cb;



#pragma vector = WDT_VECTOR  //Interval timer vector location
__interrupt void IntervalTimer(void)
{
    unsigned int a;
    a = WDTCTL;
    _cb(); // Toggle P1.0
    WDTCTL = a;
}



void wdtEnable()
{
#if defined(MSP_WDT_1000MS)
    WDTCTL = WDT_ADLY_1000;  //using ACLK from the LFXT1CLK
#elif defined(MSP_WDT_250MS)
    WDTCTL = WDT_ADLY_250;   //using ACLK from the LFXT1CLK
#elif defined(MSP_WDT_16MS)
    WDTCTL = WDT_ADLY_16;    //using ACLK from the LFXT1CLK
#else
    WDTCTL = WDT_ADLY_1_9;    //using ACLK from the LFXT1CLK
#endif

}

void wdtStart()
{
#if defined(__MSP430G2553__)
    IE1   |=  WDTIE;          //Enable the WDTIE bit
#elif defined (__MSP430FR2433__)
    SFRIE1   |=  WDTIE;          //Enable the WDTIE bit
#endif
}

void wdtReset()
{
#if defined(MSP_WDT_1000MS)
    WDTCTL = WDT_ARST_1000;  //Put WDT+ in Watch Dog Mode
#elif defined(MSP_WDT_250MS)
    WDTCTL = WDT_ARST_250;   //Put WDT+ in Watch Dog Mode
#elif defined(MSP_WDT_16MS)
    WDTCTL = WDT_ARST_16;     //Put WDT+ in Watch Dog Mode
#else
    WDTCTL = WDT_ARST_1_9;    //Put WDT+ in Watch Dog Mode
#endif
}

void wdtAttach(func_t cb)
{
    _cb = cb;
}

void wdtHold(){
    WDTCTL |= WDTPW | WDTHOLD;               // Stop watchdog timer
}

