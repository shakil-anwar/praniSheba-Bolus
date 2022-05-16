/*
 * hardwareSerial.h
 *
 *  Created on: Aug 25, 2020
 *      Author: Shuvangkar
 */

#ifndef MSP_SERIAL_H_
#define MSP_SERIAL_H_
#include <msp430.h>
#include "../mspDriver.h"
//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>


// #define BIN 2
// #define OCT 8
// #define DEC 10
// #define HEX 16

#define STR 	0
#define INT8    3
#define UINT8   4
#define INT16   5
#define UINT16  6
#define INT32   7
#define UINT32  9

#define BIN 	2
#define HEX 	16



/*********Hardware dependent Functions**********/
// void serialBegin(void);
// void serial_print_char(char c);
void SerialBegin(uint32_t baud);

void change_uart_set(uint8_t i);

void SerialPrintChar(unsigned char c);

// #define PSTR(s)   ((char*)(s))
// #define P(str)   ((const char*)(str))
// #define P(str)   (str)
char *P(char *str);
/************* generic function*****************/
void println(void);
void SerialPrint(char *str);
void SerialPrintln(char *str);

// #define SerialPrintF(str) (SerialPrint((const char*)str))
// #define SerialPrintlnF(str) (SerialPrintln((const char*)str))

void SerialPrintF(const char *str);
void SerialPrintlnF(const char *str);
/********General Serial functions**************/
void SerialPrintU8(uint8_t n);
void SerialPrintS8(int8_t n);
void SerialPrintU16(uint16_t n);
void SerialPrintS16(int16_t n);
void SerialPrintU32(uint32_t n);
void SerialPrintS32(int32_t n);


void SerialPrintlnU8(uint8_t n);
void SerialPrintlnS8(int8_t n);
void SerialPrintlnU16(uint16_t n);
void SerialPrintlnS16(int16_t n);
void SerialPrintlnU32(uint32_t n);
void SerialPrintlnS32(int32_t n);

void SerialPrintFloat(float n,uint8_t digit);
void SerialPrintlnFloat(float n,uint8_t digit);
// void serial_print_str(char *str);
// void serial_print_ulong(uint32_t n);
// void serial_print_long(int32_t n);
// void serial_print_uint(uint16_t n);
// void serial_print_int(int16_t n);
// void serial_print_int8(int8_t n);
// void serial_print_uint8(uint8_t n);

// void serial_debug_print_uint8(uint8_t n, int8_t base);
// void serial_debug_print_int8(int8_t n, int8_t base);
//void serial_print_float(float var, uint8_t precision)


void serialPrint(void *ptr, uint8_t type); //This is the common function for all type 
/**************String Utility function*****************/

#endif /* HARDWARESERIAL_H_ */
