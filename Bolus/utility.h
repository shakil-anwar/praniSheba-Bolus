/*
 * utility.h
 *
 *  Created on: Apr 21, 2021
 *      Author: sshuv
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include "mspIoT.h"

void bolusPowerDown();
void bolsuPowerUp();

uint32_t calcNextSlotUnix(uint32_t uSec, nrfNodeConfig_t *conf);
void saveUnixTime();
uint32_t readUnixTime();

/**************** FRAM Read Write Function **********************/

void FRAMWrite (uint8_t *framWritePtr, uint8_t *buf, uint16_t len);
void FRAMRead (uint8_t *framReadPtr, uint8_t *buf, uint16_t len);

#endif /* UTILITY_H_ */
