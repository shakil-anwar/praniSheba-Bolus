/*
   nrf_usr_conf.h
   Created on: Sep 2, 2020
   Author: Shuvangkar Chandra Das & Shakil Anwar
*/

#ifndef NRF_USR_CONF_H_
#define NRF_USR_CONF_H_

// #define ACK_ON
// #define ACK_PAYLOAD
// #define MAX_PAYLOAD_LEN 31
// #define AIR_DATA_SPEED  AIR_SPEED_1MB

#define RETRANSMIT_DELAY        2000
#define RETRANSMIT_COUNT        7
#define NRF_TRANSMIT_TIMEOUT    ((RETRANSMIT_DELAY*(RETRANSMIT_COUNT+1))/1000)

#define RF_CHANNALE_NO          120
#define DEFAULT_ADDR_LEN        5

#endif /* NRF_USR_CONF_H_ */
