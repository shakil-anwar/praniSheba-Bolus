/*
 * temperature.c
 *
 *  Created on: Oct 29, 2020
 *      Author: sshuv
 */

#include "mspADC.h"

#define LM20_POWER BIT0
#define TMP37_SHUT BIT1


void adcBegin()
{
    P3DIR |= (LM20_POWER | TMP37_SHUT);
    P3OUT |= (LM20_POWER | TMP37_SHUT);
    ADC10AE0 |= (BIT3 + BIT4);
    ADC10CTL1 |= INCH_4;                    // select ADC channel A4
    ADC10CTL0 = SREF_0 + ADC10SHT_3 + ADC10ON;// + REFON + REF2_5V;  // Ref -> 2.5V, 64 CLK S&H
}
uint16_t readAdc()
{
    uint16_t temp2;

    ADC10CTL1 = INCH_4;         // select channel A4 as input
    ADC10CTL0 |= ADC10SC + ENC;
    while (ADC10CTL1 & ADC10BUSY);
    temp2 = ADC10MEM;           // save ADC measure in 2nd temp variable
    ADC10CTL0 &= ~ENC;
    return temp2;
}
