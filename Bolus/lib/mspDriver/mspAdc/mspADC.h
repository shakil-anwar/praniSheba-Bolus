/*
 * temperature.h
 *
 *  Created on: Oct 29, 2020
 *      Author: sshuv
 */

#ifndef DRIVER_TEMP_TEMPERATURE_H_
#define DRIVER_TEMP_TEMPERATURE_H_
#include <msp430.h>
#include "../mspDriver.h"

void adcBegin();
uint16_t readAdc();


#endif /* DRIVER_TEMP_TEMPERATURE_H_ */
