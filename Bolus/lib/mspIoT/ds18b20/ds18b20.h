/*
 * ds18b20.h
 *
 *  Created on: Mar 21, 2022
 *      Author: Shakil Anwar
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#if defined(__MSP430G2553__) || defined(__MSP430FR2433__)
    #include <msp430.h>
    #include "../../mspDriver/mspDriver.h"
#else
    #error "nRF24_DRIVER did not find chip architecture "
#endif

#ifndef LIB_MSPIOT_DS18B20_DS18B20_H_
#define LIB_MSPIOT_DS18B20_DS18B20_H_

//**********************************************************************************************************************************************************
// Commands to handle the sensor
#define DS18B20_SKIP_ROM             0xCC
#define DS18B20_READ_SCRATCHPAD      0xBE
#define DS18B20_CONVERT_T            0x44
#define DS1820_READ_ROM              0x33

//**********************************************************************************************************************************************************
// Defines to configure pins        //SCL Pin = Pin1.3
#define DS18B20_PORT_DIR    P1DIR
#define DS18B20_PORT_OUT    P1OUT
#define DS18B20_PORT_IN     P1IN
#define DS18B20_PORT_REN    P1REN
#define DS18B20_PORT_PIN    BIT3
#define DS18B20_LO  { DS18B20_PORT_DIR |= DS18B20_PORT_PIN; DS18B20_PORT_REN &= ~DS18B20_PORT_PIN; DS18B20_PORT_OUT &= ~DS18B20_PORT_PIN; }
#define DS18B20_HI  { DS18B20_PORT_DIR |= DS18B20_PORT_PIN; DS18B20_PORT_REN &= ~DS18B20_PORT_PIN; DS18B20_PORT_OUT |= DS18B20_PORT_PIN; }
#define DS18B20_RLS { DS18B20_PORT_DIR &= ~DS18B20_PORT_PIN; DS18B20_PORT_REN |= DS18B20_PORT_PIN; DS18B20_PORT_OUT  |= DS18B20_PORT_PIN; }
#define DS18B20_IN  (DS18B20_PORT_IN & DS18B20_PORT_PIN)

//**********************************************************************************************************************************************************
// Function prototypes
void ds18b20_init_port(void);
uint8_t ds18b20_reset();
void ds18b20_write_bit(uint8_t);
uint8_t ds18b20_read_bit();
void ds18b20_write_byte(uint8_t);
uint8_t ds18b20_read_byte();
uint16_t ds18b20_read_temp_registers(void);
uint64_t ds18b20_read_rom (void);
float ds18b20_get_temp(void);
int ds18b20_get_temp_int(void);
//void show_temp(float);
void delay_us_timer1(uint16_t);
void delay_ms_timer1(uint16_t);
void set_clock_one_wire(void);
void reset_clock_one_wire(void);



#endif /* LIB_MSPIOT_DS18B20_DS18B20_H_ */
