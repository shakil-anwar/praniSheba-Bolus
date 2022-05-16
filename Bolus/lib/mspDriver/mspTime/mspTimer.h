/*
 * rtc.h
 *
 *  Created on: Sep 8, 2020
 *      Author: sshuv
 */

#ifndef DRIVER_TIMER_TIMER_H_
#define DRIVER_TIMER_TIMER_H_
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "mspTimerConf.h"

/*
 * MSP430 has two timer module. Timer0 and Timer1.
 * Each Timer(Timer0/Timer1) module has A0 and A1 sub-module. A0 has one compare match
 * and A1 has 2 compare match and one overflow interrupt.
 */
//#if defined(__MSP430G2553__)
#define TIMER_INPUTCLOCK  32768
#if RTC_CLK_DIVIDER == RTCPS__16
    #define CLOCK_DIVIDER     8
#elif RTC_CLK_DIVIDER == RTCPS__64
    #define CLOCK_DIVIDER     32
#elif RTC_CLK_DIVIDER == RTCPS__256
    #define CLOCK_DIVIDER     128
#elif RTC_CLK_DIVIDER == RTCPS__1024
    #define CLOCK_DIVIDER     512
#else
#error  Not Supported Clock Divider Value!
#endif

#define TIMER_CLOCK       (TIMER_INPUTCLOCK/CLOCK_DIVIDER)
//#endif


/***************************New Timer API Planning*****************
 * 
*******************************************************************/
typedef void (*alarmTask_t)(void);

void rtcTimerBegin();

void timerMillisStart();
void timerMillisStop();
uint32_t millis();

void rtcTimerStart();
void setSecond(uint32_t second);
uint32_t second();


void timerAlarmBegin();
bool rtcSetAlarm(uint32_t nextAlarmUtime, uint32_t repeatInterval,alarmTask_t task);
void timerSetAlarm(uint16_t intervalSec,alarmTask_t task);//call task function after this interval

void rtcTimerResetAlarm();

void timerA0C1setAlarm(uint16_t intervalSec, alarmTask_t task);
void resetTimer2Alarm();


#endif /* DRIVER_TIMER_TIMER_H_ */
