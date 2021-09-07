/*
 * Sensors.h
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */

#ifndef SENSORS_H_
#define SENSORS_H_
#include "mspDriver.h"

#define accCsLow() ({\
                        ACC_PORT_SET();\
                        ACC_DISABLE();\
                    })

void sensorsBegin();
void sensorCalibrate();
void sensorStart();

void sensorPrintLog();

extern bool _nowSend;
#endif /* SENSORS_H_ */
