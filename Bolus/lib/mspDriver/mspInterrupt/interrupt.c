/*
 * interrupt.c
 *
 *  Created on: Oct 7, 2020
 *      Author: sshuv
 */

#include "interrupt.h"
//#include <mspDriver.h>

uint8_t lpm4_int_count = 0;
uint32_t prev_lpm4_millis = 0, lpm4_int_diff = 0;

/*************Function pointer for PORT1******************/
#if defined(INT_PORT1_PIN0)
    isr_t p10_isr = NULL;
#endif
#if defined(INT_PORT1_PIN1)
    isr_t p11_isr = NULL;
#endif
#if defined(INT_PORT1_PIN2)
    isr_t p12_isr = NULL;
#endif
#if defined(INT_PORT1_PIN3)
    isr_t p13_isr = NULL;
#endif
#if defined(INT_PORT1_PIN4)
    isr_t p14_isr = NULL;
#endif
#if defined(INT_PORT1_PIN5)
    isr_t p15_isr = NULL;
#endif
#if defined(INT_PORT1_PIN6)
    isr_t p16_isr = NULL;
#endif
#if defined(INT_PORT1_PIN7)
    isr_t p17_isr = NULL;
#endif

/**********Function Pointer for PORT2***************/
#if defined(INT_PORT2_PIN0)
    isr_t p20_isr = NULL;
#endif
#if defined(INT_PORT2_PIN1)
    isr_t p21_isr = NULL;
#endif
#if defined(INT_PORT2_PIN2)
    isr_t p22_isr = NULL;
#endif
#if defined(INT_PORT2_PIN3)
    isr_t p23_isr = NULL;
#endif
#if defined(INT_PORT2_PIN4)
    isr_t p24_isr = NULL;
#endif
#if defined(INT_PORT2_PIN5)
    isr_t p25_isr = NULL;
#endif
#if defined(INT_PORT2_PIN6)
    isr_t p26_isr = NULL;
#endif
#if defined(INT_PORT2_PIN7)
    isr_t p27_isr = NULL;
#endif


/***********************PORT1 Interrupt Functions*********************************/
void attachInterrupt1(uint8_t pin, edge_e edge, isr_t isrPtr)
{
    _DINT();
    uint8_t pinMask = 1 << pin;
    P1DIR  &= ~pinMask; //set as input

#if defined (__MSP430FR2433__)
    P1SEL0 &= ~pinMask;
    P1SEL1 &= ~pinMask;
#endif

    if(edge == FALLING)
    {
        P1OUT |= pinMask; //Pull up resistor
        P1REN |= pinMask; //Enable resistor
        P1IES |= pinMask; //Trigger on falling edge
    }
    else
    {
        P1OUT &= ~pinMask;  // Pull Down resistor
        P1REN |=  pinMask;  // Enable Registor
        P1IES &= ~pinMask;  // Trigger on rising edge
    }
    switch(pin)
    {
    case 0:
#if defined(INT_PORT1_PIN0)
        p10_isr = isrPtr;
#endif
        break;
    case 1:
#if defined(INT_PORT1_PIN1)
        p11_isr = isrPtr;  
#endif
        break;
    case 2:
#if defined(INT_PORT1_PIN2)
        p12_isr = isrPtr;
#endif
        break;
    case 3:
#if defined(INT_PORT1_PIN3)
        p13_isr = isrPtr;
#endif
        break;
    case 4:
#if defined(INT_PORT1_PIN4)
        p14_isr = isrPtr;
#endif
        break;
    case 5:
#if defined(INT_PORT1_PIN5)
        p15_isr = isrPtr;
#endif
        break;
    case 6:
#if defined(INT_PORT1_PIN6)
        p16_isr = isrPtr;
#endif
        break;
    case 7:
#if defined(INT_PORT1_PIN7)
        p17_isr = isrPtr;
#endif
        break;
    }

    P1IFG &= ~pinMask;  //clear interrupt flag
    P1IE |= pinMask;    //enable Interrupt
    __enable_interrupt();
}

#pragma vector = PORT1_VECTOR
__interrupt void P1_IRQ (void)
{

#if defined(INT_PORT1_PIN0)
    if(P1IFG & BIT0)
    {
        if(p10_isr !=0){p10_isr();}
        P1IFG &= ~BIT0;
    }
#endif

#if defined(INT_PORT1_PIN1)
    if(P1IFG & BIT1)
    {
        if(p11_isr){p11_isr();}
        P1IFG &= ~BIT1;
    }
#endif

#if defined(INT_PORT1_PIN2)
    if(P1IFG & BIT2)
    {
        if(p12_isr){p12_isr();}
        P1IFG &= ~BIT2;
    }
#endif
#if defined(INT_PORT1_PIN3)
    if(P1IFG & BIT3)
    {
        if(p13_isr){p13_isr();}
        P1IFG &= ~BIT3;
    }
#endif
#if defined(INT_PORT1_PIN4)
    if(P1IFG & BIT4)
    {
       if(p14_isr){p14_isr();}
       P1IFG &= ~BIT4;
    }
#endif
#if defined(INT_PORT1_PIN5)
    if(P1IFG & BIT5)
    {
       if(p15_isr){p15_isr();}
       P1IFG &= ~BIT5;
    }
#endif
#if defined(INT_PORT1_PIN6)
    if(P1IFG & BIT6)
    {
       if(p16_isr){p16_isr();}
       P1IFG &= ~BIT6;
    }
#endif
#if defined(INT_PORT1_PIN7)
    if(P1IFG & BIT7)
    {
       if(p17_isr){p17_isr();}
       P1IFG &= ~BIT7;
    }
#endif
}

/***********************PORT2 Interrupt Functions*********************************/

void attachInterrupt2(uint8_t pin, edge_e edge, isr_t isrPtr)
{
    uint8_t pinMask = 1 << pin;
    P2DIR  &= ~pinMask; //set as input

#if defined (__MSP430FR2433__)
    P2SEL0 &= ~pinMask;
    P2SEL1 &= ~pinMask;
#endif

    if(edge == FALLING)
    {
        P2OUT |= pinMask; //Pull up resistor enable
        P2REN |= pinMask; //Enable resistor
        P2IES |= pinMask; //Trigger on falling edge
    }
    else
    {
        P2OUT &= ~pinMask;//Pull down resistor enable
        P2REN |=  pinMask;// Enable resistor
        P2IES &= ~pinMask;//Trigger on rising edge
    }

    switch(pin)
    {
    case 0:
#if defined(INT_PORT2_PIN0)
        p20_isr = isrPtr;
        prev_lpm4_millis = millis();
#endif
        break;
    case 1:
#if defined(INT_PORT2_PIN1)
        p21_isr = isrPtr;
#endif
        break;
    case 2:
#if defined(INT_PORT2_PIN2)
        p22_isr = isrPtr;
#endif
        break;
    case 3:
#if defined(INT_PORT2_PIN3)
        p23_isr = isrPtr;
#endif
        break;
    case 4:
#if defined(INT_PORT2_PIN4)
        p24_isr = isrPtr;
#endif
        break;
    case 5:
#if defined(INT_PORT2_PIN5)
        p25_isr = isrPtr;
#endif
        break;
    case 6:
#if defined(INT_PORT2_PIN6)
        p26_isr = isrPtr;
#endif
        break;
    case 7:
#if defined(INT_PORT2_PIN7)
        p27_isr = isrPtr;
#endif
        break;
    }

    P2IFG &= ~pinMask;  //clear interrupt flag
    P2IE |= pinMask;    //enable Interrupt
    __enable_interrupt();
}

#pragma vector = PORT2_VECTOR
__interrupt void P2_IRQ (void)
{
#if defined(INT_PORT2_PIN0)

    if(P2IFG & BIT0)
    {
#if defined(__MSP430G2553__)
        if(p20_isr != NULL){p20_isr();}
        P2IFG &= ~BIT0;
        delay(200);

        lpm4_int_diff = millis() - prev_lpm4_millis;
        prev_lpm4_millis = millis();

        SerialPrint(" wake_up_count: ");
        SerialPrintU8(lpm4_int_count);
        SerialPrint(" prev_lpm4_millis: ");
        SerialPrintU8(lpm4_int_diff);

        if(lpm4_int_diff >=1500)
        {
            SerialPrint("\r\n condition 1 ");
            if(lpm4_int_diff <5000)
            {
                SerialPrint("\r\n condition 2 ");
                lpm4_int_count++;
            }else
            {
                SerialPrint("\r\n condition 2/2 ");
                lpm4_int_count = 0;
            }

            if(lpm4_int_count >= 3)
            {
                SerialPrint("Exit LPM3 ");
                LPM3_EXIT;
            }
        }
        TACCR0 = TAR + 40;
#endif
    }
#endif
#if defined(INT_PORT2_PIN1)
    if(P2IFG & BIT1)
    {
        if(p21_isr != NULL){p21_isr();}
        P2IFG &= ~BIT1;
    }
#endif
#if defined(INT_PORT2_PIN2)
    if(P2IFG & BIT2)
    {
        if(p22_isr != NULL){p22_isr();}
        P2IFG &= ~BIT2;
    }
#endif
#if defined(INT_PORT2_PIN3)
    if(P2IFG & BIT3)
    {
        if(p23_isr != NULL){p23_isr();}
        P2IFG &= ~BIT3;
    }
#endif
#if defined(INT_PORT2_PIN4)
    if(P2IFG & BIT4)
    {
       if(p24_isr != NULL){p24_isr();}
       P2IFG &= ~BIT4;
    }
#endif
#if defined(INT_PORT2_PIN5)
    if(P2IFG & BIT5)
    {
       if(p25_isr != NULL){p25_isr();}
       P2IFG &= ~BIT5;
    }
#endif
#if defined(INT_PORT2_PIN6)
    if(P2IFG & BIT6)
    {
       if(p26_isr != NULL){p26_isr();}
       P2IFG &= ~BIT6;
    }
#endif
#if defined(INT_PORT2_PIN7)
    if(P2IFG & BIT7)
    {
       if(p27_isr != NULL){p27_isr();}
       P2IFG &= ~BIT7;
    }
#endif
//    serialPrint("out",STR);
}
