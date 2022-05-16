/*
 * mspWDT.h
 *
 *  Created on: Nov 24, 2020
 *      Author: Asus
 */

#ifndef MSPDRIVER_MSPWDT_MSPWDT_H_
#define MSPDRIVER_MSPWDT_MSPWDT_H_

#include "../mspDriver.h"

/*********************** set watch dog clock*****************/
#define MSP_WDT_1000MS
//#define MSP_WDT_250MS
//#define MSP_WDT_16MS
//#define MSP_WDT_1_9MS


typedef void (*func_t)(void);

void wdtEnable();
void wdtAttach(func_t cb);
void wdtStart();
void wdtReset();
void wdtHold();



#endif /* MSPDRIVER_MSPWDT_MSPWDT_H_ */
