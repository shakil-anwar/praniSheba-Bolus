/*
 * Sensors.h
 *
 *  Created on: Dec 14, 2020
 *      Author: sshuv
 */

#ifndef SENSORS_H_
#define SENSORS_H_
#include "./lib/mspDriver/mspDriver.h"
//#include "./lib/mspIoT/mspIoT.h"

#define accCsLow() ({\
                        ACC_PORT_SET();\
                        ACC_DISABLE();\
                    })


typedef enum sensorStatus_t
{
    READING_DATA = 0,
    SENDING_DATA,
    SAVING_DATA,
    IDLE,
}sensorStatus_t;

void sensorsBegin();
void sensorCalibrate();
void sensorStart();
int readOutAccFifo();

void sensorPrintLog();

extern bool _nowSend;
extern volatile uint32_t lastAccReadTime;
extern sensorStatus_t sensorStatus;
extern volatile bool accDataReady;
#endif /* SENSORS_H_ */
