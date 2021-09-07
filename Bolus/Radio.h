/*
 * Radio.h
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */

#ifndef RADIO_H_
#define RADIO_H_
#include "mspDriver.h"
#include "mspIoT.h"


void radioBegin();
void radioStart();
bool radioSendSM();

//void saveAddr(addr_t *addrPtr);
//void readAddr(addr_t *addrPtr);
//void framRead(uint32_t addr, uint8_t *buf, uint16_t len);
//void framUpdate(uint32_t addr, uint8_t *buf, uint16_t len);

bool radioSetLowPower();

extern bool _radioStartXfer;
extern nrfNodeConfig_t nrfConfig;

#endif /* RADIO_H_ */
