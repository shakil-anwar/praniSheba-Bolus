/*
 * Pin.h
 *
 *  Created on: Mar 18, 2021
 *      Author: sshuv
 */

#ifndef PIN_H_
#define PIN_H_
#include "Param.h"

#if defined(BOARD_MSP430G2_V010)

#elif defined(BOARD_MSP430FR_V010)

#define NRF_CE_PORT         P3OUT
#define NRF_CE_PIN          2
#define NRF_CSN_PORT        P2OUT
#define NRF_CSN_PIN         7
#define NRF_IRQ             0

#define FLASH_CS_PORT       P3OUT
#define FLASH_CS 	        1
#define FLASH_HOLD_PORT     P3OUT
#define FLASH_HOLD	        0

#else
#error "Board not defined"
#endif

#endif /* PIN_H_ */
