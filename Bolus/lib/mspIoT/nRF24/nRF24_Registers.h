/*
 * nRF24_register.h
 *
 *  Created on: Aug 27, 2020
 *      Author: Shuvangkar
 */

#ifndef NRF24_REGISTER_H_
#define NRF24_REGISTER_H_

/* Instructions */
#define RF24_R_REGISTER    0x00
#define RF24_W_REGISTER    0x20
#define RF24_REGISTER_MASK 0x1F
#define RF24_R_RX_PAYLOAD  0x61
#define RF24_W_TX_PAYLOAD  0xA0
#define RF24_FLUSH_TX      0xE1
#define RF24_FLUSH_RX      0xE2
#define RF24_REUSE_TX_PL   0xE3
#define RF24_R_RX_PL_WID   0x60
#define RF24_W_ACK_PAYLOAD 0xA8
#define RF24_W_TX_PAYLOAD_NOACK 0xB0
#define RF24_NOP           0xFF

/* Register Map */
#define RF24_CONFIG      0x00
#define RF24_EN_AA       0x01
#define RF24_EN_RXADDR   0x02
#define RF24_SETUP_AW    0x03
#define RF24_SETUP_RETR  0x04
#define RF24_RF_CH       0x05
#define RF24_RF_SETUP    0x06
#define RF24_STATUS      0x07
#define RF24_OBSERVE_TX  0x08
#define RF24_CD          0x09
#define RF24_RPD         0x09
#define RF24_RX_ADDR_P0  0x0A
#define RF24_RX_ADDR_P1  0x0B
#define RF24_RX_ADDR_P2  0x0C
#define RF24_RX_ADDR_P3  0x0D
#define RF24_RX_ADDR_P4  0x0E
#define RF24_RX_ADDR_P5  0x0F
#define RF24_TX_ADDR     0x10
#define RF24_RX_PW_P0    0x11
#define RF24_RX_PW_P1    0x12
#define RF24_RX_PW_P2    0x13
#define RF24_RX_PW_P3    0x14
#define RF24_RX_PW_P4    0x15
#define RF24_RX_PW_P5    0x16
#define RF24_FIFO_STATUS 0x17
#define RF24_DYNPD       0x1C
#define RF24_FEATURE     0x1D

/*******Parameter value******/
/**********Speed Flag*************/
#define AIR_SPEED_1MB   (0b000<<3)
#define AIR_SPEED_2MB   (0b001<<3)
#define AIR_SPEED_250KB (0b100<<3)
/**********TX Power***************/
#define MINUS_18_DBM    (0b00<<1)
#define MINUS_12_DBM    (0b01<<1)
#define MINUS_06_DBM    (0b10<<1)
#define ZERO_DBM        (0b11<<1)

/*********Basic Configuration*****/
#define TX_MODE         (0<<0)
#define RX_MODE         (1<<0)
#define PWR_UP          (1<<1)
#define PWR_DOWN        (0<<1)
#define PWN_DOWN_MASK   0xFD
#define CRC_EN_2_BYTE   (0b11<<2)
#define EN_ALL_INT      (0b000<<4)
#define DIS_ALL_INT     (0b111<<4)
#define RX_DR_IRQ       (1<<6)
#define TX_DS_IRQ       (1<<5)
#define MAX_RT_IRQ      (1<<4)

/*********Acknowledgment**********/
#define PIPE0_ACK       (1<<0)
#define PIPE1_ACK       (1<<1)
#define PIPE2_ACK       (1<<2)
#define PIPE3_ACK       (1<<3)
#define PIPE4_ACK       (1<<4)
#define PIPE5_ACK       (1<<5)

/******** Status Register**********/
#define MAX_RT_BIT     4
#define TX_DS_BIT      5
#define RX_DR_BIT      6

#define MAX_RT         (1<<MAX_RT_BIT)
#define TX_DS          (1<<TX_DS_BIT)
#define RX_DR          (1<<RX_DR_BIT)

#define PIPE_MASK       0x07

/*******FIFO Status Mask*********/
#define RX_FULL        (1<<1)
#define RX_EMPTY       (1<<0)
#define TX_FULL        (1<<5)
#define TX_EMPTY       (1<<4)

/*********Feature Register*******/
#define EN_DLP_BIT 		2
#define EN_DLP         (1<<2)
#define EN_ACK_PAY     (1<<1)
#define EN_DYN_ACk     (1<<0)
/***********Transmit delay and cocunt**/
#define RETRANSMIT_OPCODE (((RETRANSMIT_DELAY-250)/250)<<4)


typedef enum air_speed_t
{
    SPEED_1MB = AIR_SPEED_1MB,
    SPEED_2MB = AIR_SPEED_2MB,
    SPEED_250KB = AIR_SPEED_250KB
}air_speed_t;

typedef enum power_t
{
    POWER_MINUS_18_DBM =  MINUS_18_DBM,
    POWER_MINUS_12_DBM =  MINUS_12_DBM,
    POWER_MINUS_06_DBM = MINUS_06_DBM,
    POWER_ZERO_DBM     = ZERO_DBM
}power_t;


#endif /* NRF24_REGISTER_H_ */
