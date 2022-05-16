/*
 * msp430f2553_I2C.h
 *	13/07/2020: Updated to receive multiple byte
 *  Created on: Jun 26, 2020
 *      Author: Md Shakil Anwar
 */

/*********************************************************************************
 *This library is only for msp430f2553 I2C/ Two Wire Synchronous communication communication
 * Default sync speed 100KHz for sync_clock_speed = 0;
 * if You don't want to define any value follow the following default list
 * TXByteCounter =1;
 *********************************************************************************
 */


#ifndef INCLUDE_MSP430F2553_I2C_H_
#define INCLUDE_MSP430F2553_I2C_H_

#include <msp430.h>
#include "../mspDriver.h"

#define MSP_I2C_TIMEOUT     1000


void i2c_init(void);
int i2c_senddata(char *data, int len);
int i2c_readdata(char *data, int len);
int i2c_start(char addr, int read);
int i2c_start2(char addr, int read);
int i2c_stop(void);

int i2cReadGyroData(char addr,char *data, int len);
int i2cWriteGyroData(char addr,char *data, int len);




//bool initiate_I2C_master(uint8_t slaveAddress, uint16_t sync_clock_speed);	/// Initiation of communication only need 7 bit salve address and SPI Communication speed
//uint8_t I2C_transmit_master(uint8_t *I2C_dataToSend, int TXByteCounter);		/// To transmit data length of data and data pointer need to be assigned
//uint8_t I2C_receive_master(uint8_t *I2C_dataToSend, int TXByteCounter);	/// this function receives 8 bit data in output whenever it is called
//void I2C_set_slave_addr(uint8_t slaveAddress);
//void I2C_Read_EEProm(unsigned int memory_address, char * data, unsigned char DataLength );
//
//__interrupt void USCIAB0RX_ISR(void);
//
//__interrupt void USCIAB0TX_ISR(void);




#endif /* INCLUDE_MSP430F2553_I2C_H_ */
