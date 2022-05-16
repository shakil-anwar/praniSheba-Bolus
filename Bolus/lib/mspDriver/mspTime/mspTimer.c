/*
 * rtc.c
 *
 *  Created on: Sep 8, 2020
 *      Author: sshuv
 */
#include "mspTimer.h"
#include "../mspDriver.h"


//#include "mspDriver.h"


void alarmTrack();

volatile uint32_t _ms;
volatile uint32_t _sec;

/********Alarm Variable****************/
volatile uint8_t _alarmFlag;
volatile uint8_t _alarmFirstRem;
volatile uint8_t _alarmFullMaxCycle;
volatile uint8_t _alarmCounter;
volatile uint8_t _alarmLastRem;
volatile uint32_t _rtcAlarmRepeatInterval;
volatile uint16_t _alarm2Time; // for timerA0 comparator 1
volatile uint16_t _alarm2counter;

volatile alarmTask_t _task;
volatile alarmTask_t _alarm2Task;

#if defined(__MSP430G2553__)
//Timer0_A0 interrupt service routine
#pragma vector = TIMER0_A0_VECTOR
__interrupt void timerA0_isr()
{
    //This interrupt will loose few ms in 1 second
    TACCR0 += MILLIS_CAPTURE_VALUE;
    _ms += MILLIS_UPDATE_INTERVAL;
//    serialPrint(".", STR);
}

//Timer0 A1 interrupt service routine.
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TimerA1_isr()
{
//    serial_print("\r\nTAIV: ");serial_print_ulong(TAIV);
    switch (TAIV)
    {
    //TACCR1 capture goes here
    case 2:
        rtcTimerResetAlarm();
        if(_task)
        {
            _task();
        }
        break;
    //TACCR2 capture goes here
    case 4:
        //        TACCR2 += CLOCK_CAPTURE_VAL2;
        //        counter2 += CLOCK2_INTERVAL_SEC;
        break;
    case 10: //Timer Overflow after 2^16 counter
        _sec += RTC_UPDATE_INTERVAL;
        alarmTrack();
    default:
        break;
    }
}

#elif defined (__MSP430FR2433__)

#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
{

    switch(__even_in_range(RTCIV, RTCIV_RTCIF))
    {
        case RTCIV_NONE : break;            // No interrupt pending
        case RTCIV_RTCIF:                   // RTC Overflow
            // Toggle LED on P1.0
//            P1OUT ^= BIT0;
#if RTC_CLK_DIVIDER == RTCPS__16
            _sec +=32;
#elif RTC_CLK_DIVIDER == RTCPS__64
            _sec +=128;
#elif RTC_CLK_DIVIDER == RTCPS__256
            _sec +=512;
#elif RTC_CLK_DIVIDER == RTCPS__1024
            _sec +=2048;
#else
#error  Not Supported Clock Divider Value!
#endif
            // Store P1OUT value in backup memory register
            *(unsigned long *)BKMEM_BASE = _sec;
            break;
        default:          break;
    }
}


#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{

    if(_alarmFlag)
    {
        alarmTrack();
    }else{

        timerSetAlarm(_rtcAlarmRepeatInterval,_task);
        if(_task)
        {
            _task();
        }

        __low_power_mode_off_on_exit();
//        __bis_SR_register_on_exit(LPM3);
    }
}

#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer1_A (void)
{


    if(TA0IV & TA0IV_TACCR1)
    {
        if(_alarm2Task)
        {
            _alarm2Task();
        }
        resetTimer2Alarm();
    }

}



#endif

// *************** CODE FOR MSP430G2553***********//
#if defined(__MSP430G2553__)
void rtcTimerBegin()
{

    // clock ACLK = 32KHz | Clock divider 8 | Continuous mode
    TACTL = TASSEL_1 | ID_3 | MC_2;
    TAR = 0; //Reset TimerA Counter
    //    TACTL |= TASSEL_1;
    //    TACTL |= ID_3;
    //    TACTL |= MC_2;
}

void timerMillisStart()
{

    _ms = 0;
    TACCR0 = MILLIS_CAPTURE_VALUE - 1;
    TACCTL0 |= CCIE; // Enable TACCR0 Interrupt

}

void timerMillisStop()
{
    TACCTL0 &= ~CCIE;
}

uint32_t millis()
{
    return _ms;
}

void rtcTimerStart()
{
    _sec = 0;
    TACTL |= TAIE; //Enable Overflow Interrupt.
    //Timer overflows after 65553 counts(16S)
}

void setSecond(uint32_t second)
{
    _sec = second;
}
uint32_t second()
{
    uint8_t rem = TAR>>12;
    return (_sec + rem);
}

void timerSetAlarm(uint16_t intervalSec, alarmTask_t task)
{
    uint16_t temp;
    temp = (TAR >> 12);//TAR/One_sec counter
    _alarmFirstRem = (uint8_t)temp;
    if(_alarmFirstRem)
    {
        _alarmFirstRem = ((uint8_t)RTC_UPDATE_INTERVAL - _alarmFirstRem);
    }
//    SerialPrint("\r\n_alarmFirstRem: ");SerialPrintU32(_alarmFirstRem);

    intervalSec = intervalSec - _alarmFirstRem;
//    SerialPrint("\r\nintervalSec: ");SerialPrintU32(intervalSec);

    uint8_t remFlag = (_alarmFirstRem ? 1:0);


    temp = intervalSec >> 4;
    _alarmFullMaxCycle = (uint8_t)temp + remFlag;
//    _alarmFullMaxCycle = (uint8_t)(intervalSec / RTC_UPDATE_INTERVAL) + remFlag; // 1 comes from the firstRem
//    SerialPrint("\r\n_alarmFullMaxCycle: ");SerialPrintU32(_alarmFullMaxCycle);

    temp = intervalSec - (intervalSec >> 4)*16;
    _alarmLastRem = (uint8_t)temp;
//    _alarmLastRem = (uint8_t)(intervalSec% RTC_UPDATE_INTERVAL);
//    SerialPrint("\r\n_alarmLastRem: ");SerialPrintU32(_alarmLastRem);
    rtcTimerResetAlarm();

    _task = task;
//     _alarmFlag = true;
}

void rtcTimerResetAlarm()
{
    _alarmCounter = 0;
    //disable TCCR1 Interrupt
#if defined(__MSP430G2553__)
    if(_alarmFullMaxCycle)
    {
        TACCTL1 &= ~CCIE;
        _alarmFlag = true;
    }else{
        TACCR1 += (_alarmLastRem<<12) - 1;
//        SerialPrint("\r\nTACCR1: ");SerialPrintU32(TACCR1);
        TACCTL1 |= CCIE; //Enable CCR1 Interrupt
        _alarmFlag = false;
    }
#else
#error No Supported Board
#endif
}
void alarmTrack()
{
    if (_alarmFlag)
    {
        _alarmCounter++;
//        SerialPrint("\r\nCounter: ");SerialPrintU32(_alarmCounter);
        if (_alarmCounter >= _alarmFullMaxCycle)
        {
//            SerialPrint("\r\nTrigger: ");SerialPrintU32(_alarmCounter);
            //Start TCCR1 Interrupt for _alarmLastRem
#if defined(__MSP430G2553__)
            TACCR1 += (_alarmLastRem<<12) - 1;
            SerialPrint("\r\nTACCR1: ");SerialPrintU32(TACCR1);
            TACCTL1 |= CCIE; //Enable CCR1 Interrupt
#else
#error No Supported Board
#endif
            _alarmFlag = false;
        }
    }
}

#elif defined (__MSP430FR2433__)

// *************** CODE FOR MSP430FR2433***********//

void rtcTimerBegin()
{
    RTCCTL &= ~RTCIE;

    P2SEL0 |= BIT0 | BIT1;                  // set XT1 pin as second function

#if RTC_CLK_DIVIDER == RTCPS__16
    RTCCTL = RTCSS1 | RTCPS__16;            // select XT1CLK, Divider 1/16,
#elif RTC_CLK_DIVIDER == RTCPS__64
    RTCCTL = RTCSS1 | RTCPS__64;            // select XT1CLK, Divider 1/64,
#elif RTC_CLK_DIVIDER == RTCPS__256
    RTCCTL = RTCSS1 | RTCPS__256;            // select XT1CLK, Divider 1/64,
#elif RTC_CLK_DIVIDER == RTCPS__1024
    RTCCTL = RTCSS1 | RTCPS__1024;            // select XT1CLK, Divider 1/64,
#else
#error  Not Supported Clock Divider Value!
#endif
    RTCMOD = 0xFFFF;

    _alarm2Time = 0;
    _alarm2Task = NULL;
}

void rtcTimerStart()
{
    _sec = 0;
    RTCMOD = 0xFFFF;
    RTCCTL |= RTCSR | RTCIE;                // reset RTCCNT and Enable RTC interrupt
    RTCIV;                                  // Clear Interrupt flag
    //Timer overflows after 65553 counts(16S)
}

void timerMillisStart()
{

    _ms = 0;
    RTCCTL |=  RTCSR | RTCSS1 | RTCIE;                // reset RTCCNT and Enable RTC interrupt
    RTCIV;                                  // Clear Interrupt flag
}

void timerMillisStop()
{
    RTCCTL &= ~RTCSS1;
}

uint32_t millis()
{
    uint32_t tTime2;
//    tTime = unixTime*1000UL + (uint32_t)((RTCCNT>>1)/1.024);
    _ms = (_sec<<10)-(_sec*24);
    tTime2 = (uint32_t)RTCCNT;
#if RTC_CLK_DIVIDER == RTCPS__16
    //    tTime2 =(uint32_t)((RTCCNT>>1)/1.024);
    //    tTime2 =(uint32_t)((RTCCNT>>11)*1000);
//        tTime2 = (uint32_t)RTCCNT;
        tTime2 =(tTime2*1000)/2048;
    #elif RTC_CLK_DIVIDER == RTCPS__64
    //    tTime2 =(uint32_t)(RTCCNT/0.512);
    //    tTime2 =(uint32_t)((RTCCNT>>9)*1000);
        tTime2 =(tTime2*1000)/512;
    #elif RTC_CLK_DIVIDER == RTCPS__256
    //    tTime2 =(uint32_t)(RTCCNT/0.128);
    //    tTime2 =(uint32_t)((RTCCNT>>7)*1000);
        tTime2 =(tTime2*1000)/128;
    #elif RTC_CLK_DIVIDER == RTCPS__1024
    //    tTime2 =(uint32_t)(RTCCNT/0.032);
//        tTime2 =(uint32_t)((RTCCNT>>5)*1000);
        tTime2 =(tTime2*1000)/32;
#else
#error  Not Supported Clock Divider Value!
#endif
    _ms +=tTime2;

    return _ms;
}


void setSecond(uint32_t second)
{
    RTCCTL &= ~RTCIE;
    _sec = second;
    RTCIV;
    RTCCTL |= RTCSR | RTCIE;                // reset RTCCNT and Enable RTC interrupt
//    timerMillisStart();
}

uint32_t second()
{
    uint32_t rem ;

#if RTC_CLK_DIVIDER == RTCPS__16
    rem = _sec + (uint32_t)((RTCCNT>>11));
#elif RTC_CLK_DIVIDER == RTCPS__64
    rem = _sec + (uint32_t)((RTCCNT>>9));
#elif RTC_CLK_DIVIDER == RTCPS__256
    rem = _sec + (uint32_t)((RTCCNT>>7));
#elif RTC_CLK_DIVIDER == RTCPS__1024
    rem = _sec + (uint32_t)((RTCCNT>>5));
#else
#error  Not Supported Clock Divider Value!
#endif
    return rem;
}

bool rtcSetAlarm(uint32_t nextAlarmUtime, uint32_t repeatInterval, alarmTask_t task)
{
    uint32_t nextAlarmTime = second();
    if(nextAlarmUtime>nextAlarmTime)
    {
        _rtcAlarmRepeatInterval = repeatInterval;
        nextAlarmTime = nextAlarmUtime - nextAlarmTime;
        timerSetAlarm((uint16_t)nextAlarmTime, task);
        return true;
    }
    return false;

}

void timerSetAlarm(uint16_t intervalSec, alarmTask_t task)
{
    TA0CTL &= 0xFFCF;                             // HALT Timer0_A3
    TA0CTL  = TASSEL_1 | ID_3| TACLR ;                  // Select ACLK, Clock Devider 8, Clear TA0R

    TA0R = 0x00;
    uint16_t temp;
    temp = (TA0R >> 12);//TAR/One_sec counter
    _alarmFirstRem = (uint8_t)temp;
    if(_alarmFirstRem)
    {
        _alarmFirstRem = ((uint8_t)RTC_UPDATE_INTERVAL - _alarmFirstRem);
    }
//    SerialPrint("\r\n_alarmFirstRem: ");SerialPrintU32(_alarmFirstRem);

    intervalSec = intervalSec - _alarmFirstRem;

    uint8_t remFlag = (_alarmFirstRem ? 1:0);


    temp = intervalSec >> 4;
    _alarmFullMaxCycle = (uint8_t)temp + remFlag;
    SerialPrint("\r\n_aFMC:");SerialPrintU8(_alarmFullMaxCycle);

    temp = intervalSec - (intervalSec >> 4)*16;
    _alarmLastRem = (uint8_t)temp;
    _task = task;

    TA0CCTL0 |= CCIE;                             // TACCR0 interrupt enabled
    rtcTimerResetAlarm();                                 // reset alarm flag and counters
}

void rtcTimerResetAlarm()
{
    _alarmCounter = 0;

    //disable TCCR1 Interrupt
#if defined(__MSP430FR2433__)
    TA0CTL &= 0xFFCF;                             // HALT Timer0_A3
    TA0CTL |= TACLR;                              // Clear Timer0_A3 counter
    // Keep TimerA0_Comparator 1 in sync;
    if(_alarm2Time)
    {
        TA0CCR1 = _alarm2Time;
    }

    //
    if(_alarmFullMaxCycle)
    {
        TA0CCR0 = 0xFFFF;                             // TimerA countFull Cycle.
        _alarmFlag = true;
    }else{
        TA0CCR0 = (_alarmLastRem<<12) - 1;            // Set last remider Second
        _alarmFlag = false;
    }
    TA0CTL |= MC__CONTINUOUS;                             // Enable Up Count Mode
#else
#error No Supported Board
#endif
}
void alarmTrack()
{
    if (_alarmFlag)
    {
        _alarmCounter++;
        if (_alarmCounter >= _alarmFullMaxCycle)
        {
#if defined(__MSP430FR2433__)
            TA0CTL &= 0xFFCF;                             // HALT Timer0_A3
            TA0CCR0 = (_alarmLastRem<<12) - 1;            // Set last remider Second
            TA0CTL |= MC__CONTINUOUS;                             // Enable Up Count Mode
#else
#error No Supported Board
#endif
            _alarmFlag = false;
        }
    }
}


void timerA0C1setAlarm(uint16_t intervalSec, alarmTask_t task)  //intervalSet in millisecond multiple of 0.125 sec
{
    TA0CTL &= 0xFFCF;
    _alarm2Time = 512*intervalSec;                // 0.125s*intervalSec
    SerialPrint("Alarm2 Interval Time: "); SerialPrintlnU16(_alarm2Time);
    _alarm2Task = task;
    TA0CCR1 = TA0R;
    TA0CCTL1 |= CCIE;                             // TACCR1 interrupt enabled
    resetTimer2Alarm();
}


void resetTimer2Alarm()
{
    TA0CTL &= 0xFFCF;                             // HALT Timer0_A3
    TA0CCR1 += _alarm2Time;
    TA0CTL |= MC__CONTINUOUS;                             // Enable Up Count Mode
//    SerialPrintln("Alarm2 Reset");
}


#endif




void timerAlarmBegin()
{
    _alarmFlag = false;
    _alarmCounter = 0;
    _task = NULL;
}


// TACCR1 = CLOCK_CAPTURE_VAL2-1;
//     TACCTL1 |= CCIE; //Enable CCR1 Interrupt
//
// void timer0_A0_begin()
// {
//     ms = 0;
//     TACCR0 = CLOCK_CAPTURE_VAL1-1;
//     TACCTL0 |= CCIE;                // Enable TACCR0 Interrupt
// }
//
// void timer0_A0_stop()
// {
//     TACCTL0 &= !CCIE;
// }
//
// void timer0_A1_begin()
// {
//     unix_sec = 0;
// #if defined(ENABLE_CCR1)
//     TACCR1 = CLOCK_CAPTURE_VAL2-1;
//     TACCTL1 |= CCIE; //Enable CCR1 Interrupt
// #endif
// #if defined(ENABLE_CCR2)
//     TACCR2 = CLOCK_CAPTURE_VAL2-1;
//     TACCTL2 |= CCIE; //Enable CCR2 Interrupt
// #endif
//
//     TACTL |= TAIE; //Enable Overflow Interrupt
//
// }




