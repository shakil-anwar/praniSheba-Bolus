/*
   NRF24_Driver.h

    Created on: Aug 27, 2020
        Author: Shuvangkar
*/

#ifndef NRF24_DRIVER_H_
#define NRF24_DRIVER_H_

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
    #error "nRF24_DRIVER did not find chip architecture "
#endif
// Default Core Library
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nRF24_User_Conf.h"
#include "nRF24_Registers.h"


#if defined(ARDUINO_ARCH_AVR) || defined(__MSP430G2553__)
typedef uint8_t gpio_pin;
#elif defined(__MSP430FR2433__)
typedef uint16_t gpio_pin;
#endif

#define nrf_csn_low()     (*_csnPort &= ~(1<< _csnPin))
#define nrf_csn_high()    (*_csnPort |=  (1<< _csnPin))

#define nrf_ce_high()  (*_cePort |= (1 << _cePin));
#define nrf_ce_low()   (*_cePort &= ~(1 << _cePin));

#if defined(ARDUINO_ARCH_AVR)
    #define nrf_ce_read()  (*(_cePort-2) & (1 << _cePin));
#elif defined(__MSP430G2553__)
    #define nrf_ce_read()  (*(_cePort-1) & (1 << _cePin));
#elif defined(__MSP430FR2433__)
    #define nrf_ce_read()  (*(_cePort-1) & (1 << _cePin));
#else
 #error "chip support not found"
#endif

uint8_t read_register(uint8_t addr);
void write_register(uint8_t addr, uint8_t data);

void write_bytes_in_register(uint8_t addr, uint8_t *payload, uint8_t len);
uint8_t *read_bytes_in_register(uint8_t addr, uint8_t *bucket, uint8_t len);

void set_reg_bit(uint8_t reg, uint8_t bitMask);
void clear_reg_bit(uint8_t reg, uint8_t bitMask);

void nrf_flush_tx();
void nrf_flush_rx();

void nrf_set_tx_addr(uint8_t *addr, uint8_t len);
void nrf_set_rx_addr(uint8_t pipe, uint8_t *addr, uint8_t len);


void nrf_set_addr_width(uint8_t width);
// #define nrf_set_addr_width(width) write_register(RF24_SETUP_AW, (width - 2) & 0b00000011);

void nrf_set_tx_dbm_speed(uint8_t);
// #define nrf_set_tx_dbm_speed(dbm_speed)  write_register(RF24_RF_SETUP, (dbm_speed) & 0x2F);

void dlp_enable();
void dlp_disable();
// #define dlp_enable() write_register (RF24_FEATURE,EN_DLP)
// #define dlp_disable() clear_reg_bit(RF24_FEATURE,EN_DLP)

void dynpd_disable(uint8_t pipe); //dynamic payload disable on pipe
void dynpd_enable(uint8_t pipe); //dynamic payload enable on pipe

// uint8_t nrf_debug_register(uint8_t addr);
void nrfPrintBuffer(void *ptr, uint8_t len);


uint8_t checksum(void *buffer, uint8_t len);

/*******Extern global vars******/
extern volatile uint8_t _nrfStatusReg;
extern volatile bool _nrfDebug;
extern volatile gpio_pin *_cePort;
extern volatile gpio_pin _cePin;
extern volatile gpio_pin *_csnPort;
extern volatile gpio_pin _csnPin;

#endif /* NRF24_DRIVER_H_ */


