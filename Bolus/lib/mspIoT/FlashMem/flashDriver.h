#ifndef _FLASH_DRIVER_H_
#define _FLASH_DRIVER_H_

#include "FlashRegisters.h"
#if defined(ARDUINO_ARCH_AVR)
    #include <Arduino.h>
    #if defined(PROD_BUILD)
        #include "../arduinoCwrapper/Serial.h"
        #include "../arduinoCwrapper/spi_driver.h"
    #else
        #include "spi_driver.h"
        #include "Serial.h"
    #endif
#elif defined(ARDUINO_ARCH_SAM)
    #include <Arduino.h>
#elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
    #include <msp430.h>
    #include "../../mspDriver/mspDriver.h"
#else
    #error "flashMem did not find chip architecture "
#endif

// Default Core Library
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


/***********************************/
#define NON_VOLATILE 0
#define VOLATILE 1

#define BUSY_BIT 0
#define WRITING_BIT 1
#define WPS_BIT 2

#if defined(ARDUINO_ARCH_AVR) || defined(__MSP430G2553__)
typedef uint8_t gpio_t;
#elif defined(__MSP430FR2433__)
typedef uint16_t gpio_t;
#endif
/****************HArdware specific functions**************/

void flashSetPin(volatile gpio_t *csPort, volatile gpio_t csPin);
void csLow();
void csHigh();
uint8_t _readStatusReg(uint8_t regNo);
bool _getStatus(uint8_t bit);
uint8_t _readStatusReg1();
bool _writeEnable(uint8_t memType);
void _writeEnablefast();
void _writeDisable();
void _busyWait();
void _spiSendAddr(uint32_t addr);
void _writeStatusReg(uint8_t reg, uint8_t value, uint8_t memType);
uint16_t mod16(uint16_t divident, int8_t divisorPower);

#endif
