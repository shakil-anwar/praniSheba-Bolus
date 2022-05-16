/*
 * timer_user_conf.h
 *
 *  Created on: Sep 9, 2020
 *      Author: sshuv
 */

#ifndef DRIVER_TIMER_TIMER_USER_CONF_H_
#define DRIVER_TIMER_TIMER_USER_CONF_H_
#include "mspTimer.h"

#if defined(__MSP430G2553__)

#define RTC_UPDATE_INTERVAL         CLOCK_DIVIDER*2
//#define RTC_PERIOD_CAPTURE_VALUE    ((TIMER_CLOCK*RTC_UPDATE_INTERVAL)/4)
#define ONE_SEC_COUNTER_VALUE       (65536/RTC_UPDATE_INTERVAL)

#define MILLIS_UPDATE_INTERVAL      5
#define MILLIS_CAPTURE_VALUE        ((TIMER_CLOCK*MILLIS_UPDATE_INTERVAL)/(1000*1))

#elif defined (__MSP430FR2433__)

#define RTC_CLK_DIVIDER             RTCPS__16
#define RTC_UPDATE_INTERVAL         CLOCK_DIVIDER*2

#endif


#endif /* DRIVER_TIMER_TIMER_USER_CONF_H_ */
