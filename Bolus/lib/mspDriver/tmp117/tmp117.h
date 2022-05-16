#ifndef TMP_117_DRIVER_H_
#define TMP_117_DRIVER_H_

#include <msp430.h>
#include "../mspDriver.h"
#include "tmp117_register.h"

#define DEVICE_ID_VALUE             0x0117          // Value found in the device ID register on reset (page 24 Table 3 of datasheet)
#define TMP117_RESOLUTION           0.0078125f    // Resolution of the device, found on (page 1 of datasheet)
#define CONTINUOUS_CONVERSION_MODE  0b00 // Continuous Conversion Mode
#define ONE_SHOT_MODE               0b11              // One Shot Conversion Mode
#define SHUTDOWN_MODE               0b01
#define AVERAGE_CONVERSION_TIME     0b001
#define AVERAGE_COUNT               0b01
#define TMP117_ADDR                 0x48

void tmp117_init(void);
int tmp117_gettemp(void);
int read_reg(int address, int reg);
int write_reg(int address, int reg, int value);
void tmp117_shutdown();


#endif
