/*
 * SPI.h
 *
 *  Created on: Aug 26, 2020
 *      Author: Shuvangkar
 */

#ifndef MSP_SPI_H_
#define MSP_SPI_H_

#include <msp430.h>
#include "../mspDriver.h"

//#define CS_PORT_DIR     P3DIR
//#define CS_PORT_OUT     P3OUT
//#define CS_PIN          BIT6
//
//
//
////#define CS_PORT_DIR     P1DIR
////#define CS_PORT_OUT     P1OUT
////#define CS_PIN          BIT4
//
//
//#define CS_PORT_SET() (CS_PORT_DIR |= CS_PIN)
//#define CS_ENABLE() (CS_PORT_OUT &= ~CS_PIN)
//#define CS_DISABLE() (CS_PORT_OUT |= CS_PIN)


void spi_begin(uint32_t speed);
// void spi_begin(void);
uint8_t spi_transfer(uint8_t reg);
uint16_t spi_transfer16(uint16_t input16);

#endif /* SPI_H_ */
