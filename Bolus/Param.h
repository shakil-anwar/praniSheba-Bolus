/*
 * Param.h
 *
 *  Created on: Dec 15, 2020
 *      Author: sshuv
 */

#ifndef PARAM_H_
#define PARAM_H_

/********************Build flags****************************/
//#define BOARD_MSP430G2_V010
#define BOARD_MSP430FR_V010
/************************************************************
 *                   Device Parameter
 ***********************************************************/
#define DEVICE_ID               1005
#define DEVICE_TYPE             1
#define RF_ACTIVATION_RETRY     10
//#define RESET_MEMORY
//#define FACTORY_RESET
/********************Buffer Parameters**********************/
#define TOTAL_PAYLOAD_BUFFER        20
#define BOLUS_SAMPLE_IN_PACKET      8

#define TOTAL_FLASH_BUFFER          800

/***************Time Parameters***********************/
#define DATA_TRANSMIT_INTERVAL      40
/***************Communication Parameter****************/
#define SPI_SPEED       100000UL
#define SERIAL_SPEED    115200

/************** NRF FRAM Address for Info Saving ******************/
#define FRAM_TEST_START              0x1800
#define FRAM_NRF_DATA_SEND_ADDRESS   0x1832
#define FRAM_TIME_SAVE_ADDRESS       0x1852


#endif /* PARAM_H_ */
