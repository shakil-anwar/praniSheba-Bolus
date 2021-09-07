/*
 * All.h
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */
#ifndef ALL_H_
#define ALL_H_
#include "mspDriver.h"
#include "mspIoT.h"
#include "memory.h"
#include "Sensors.h"
#include "Radio.h"
#include "pin.h"
#include "Schema.h"

//void setupAll();
//void setupMspCore();
//void setupChip();
//void setupInt();
//void startDevice();


bool isHardwareOk();
void alarmTask();
void mcuBasicSetup();
void setupBasic();
bool syncDevice();
void routineTask();

void printRunLog();

extern uint32_t _nowSec;
extern uint32_t _prevSec;

#endif /* ALL_H_ */
