/*
 * hardwareSerial.c
 *
 *  Created on: Aug 25, 2020
 *      Author: Shuvangkar
 */

//#include "mspDriver.h"

#include "Serial.h"
//#include <Serial.h>
// #include "my_string.h"


//char *tx_ptr = NULL;

//#define BAUD 38400
#define BAUD 115200

#if defined(__MSP430G2553__)

#if BAUD == 9600
    #define UCA0BR0_VAL     104
    #define UCA0BR1_VAL     0
    #define UCA0MCTL_VAL    UCBRS_1
#elif BAUD == 19200
    #define UCA0BR0_VAL     52
    #define UCA0BR1_VAL     0
    #define UCA0MCTL_VAL    UCBRS_0
#elif BAUD == 38400
    #define UCA0BR0_VAL     26
    #define UCA0BR1_VAL     0
    #define UCA0MCTL_VAL    UCBRS_0
#elif BAUD == 56000
    #define UCA0BR0_VAL     17
    #define UCA0BR1_VAL     0
    #define UCA0MCTL_VAL    UCBRS_7
#elif BAUD == 115200
    #define UCA0BR0_VAL     8
    #define UCA0BR1_VAL     0
    #define UCA0MCTL_VAL    UCBRS_6
#else
    #error BAUD RATE NOT SUPPORTED
#endif

#elif defined (__MSP430FR2433__)

#if BAUD == 9600
    #define UCOS16_VAL     1
    #define UCBRS0_VAL     0x20
    #define UCBRF0_VAL     8
    #define UCA0BRW_VAL    6
#elif BAUD == 19200
    #define UCOS16_VAL     1
    #define UCBRS0_VAL     0x2
    #define UCBRF0_VAL     4
    #define UCA0BRW_VAL    3
#elif BAUD == 38400
    #define UCOS16_VAL     1
    #define UCBRS0_VAL     0x0
    #define UCBRF0_VAL     10
    #define UCA0BRW_VAL    1
#elif BAUD == 56000
    #define UCOS16_VAL     0
    #define UCBRS0_VAL     0x4A
    #define UCBRF0_VAL     0
    #define UCA0BRW_VAL    17
#elif BAUD == 115200
    #define UCOS16_VAL     0
    #define UCBRS0_VAL     0x08
    #define UCBRF0_VAL     0
    #define UCA0BRW_VAL    9
#else
    #error BAUD RATE NOT SUPPORTED
#endif

#define UCA0MCTLW_VAL   ((UCBRS0_VAL<<8) | (UCBRF0_VAL<<4) | UCOS16_VAL)

#endif

#if defined(__MSP430G2553__)
void SerialBegin(uint32_t baud)
{
    UCA0CTL1 |= UCSWRST;        //USCI logic held in reset state means disabled
    P1SEL |= BIT1+BIT2;         //PxSEL & PxSEL2 both registers select the pin attribute
    P1SEL2 |= BIT1+BIT2;        //P1.1 = RXD | P1.2 = TXD

    UCA0CTL1 |= UCSSEL_2;       //SMCLK clock source

    // switch(baud)
    // {
    // 	case 9600:
    // 		UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
    // 		UCA0BR1 = UCA0BR1_VAL;      //ignore
    // 		UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.
    // 	break;
    // 	case 19200:
    // 		UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
    // 		UCA0BR1 = UCA0BR1_VAL;      //ignore
   	// 	    UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.
    // 	break;
    // 	case 38400:
    // 		UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
    // 		UCA0BR1 = UCA0BR1_VAL;      //ignore
    // 		UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.
    // 	break;
    // 	case 56000:
    // 		UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
    // 		UCA0BR1 = UCA0BR1_VAL;      //ignore
    // 		UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.
    // 	break;
    // 	case 115200:
    // 		UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
   	// 	    UCA0BR1 = UCA0BR1_VAL;      //ignore
   	// 	    UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.
    // 	break;
    // }

    UCA0BR0 = UCA0BR0_VAL;      // 104 defined in datasheet for 9600 baudrate while SMCLK is selected
    UCA0BR1 = UCA0BR1_VAL;      //ignore
    UCA0MCTL = UCA0MCTL_VAL;    //modulation stage comes from baud rate table.


    UCA0CTL0 = 0x00;            //ignore
    UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine

//    IE2 |= UCA0TXIE;                  // Enable the Transmit interrupt
//    _BIS_SR(GIE);                     // Enable the global interrupt
}

void SerialPrintChar(unsigned char c)
{
    UCA0TXBUF = c;
    while(UCA0STAT & UCBUSY);
}

void SerialPrint(char *str)
{
    while(*str)
    {
        UCA0TXBUF = *str;
        while(UCA0STAT & UCBUSY); //UCBUSY Flag = 1(Transmit/Receive) | 0(Inactive)
        str++;
    }
}

#elif defined (__MSP430FR2433__)

void SerialBegin(uint32_t baud)
{
    UCA0CTL1 = UCSWRST;        //USCI logic held in reset state means disabled

//    uint32_t brDiv,brMod;
//
//    brDiv = (1000000<<4)/baud;
//    brMod = brDiv & 0xFFF0;
//    brDiv >>=8;
//    brMod >>=4;
//
//    uint8_t overSample, firstMod,secMod;
//
//    if(brMod > 0x0F)
//    {
//        overSample = 1;
//    }else{
//        overSample = 0;
//    }
//    firstMod = (uint8_t) (brMod & 0x0F);
//    secMod = (uint8_t) (brMod >> 4);

    P1SEL0 |= BIT4+BIT5;         //PxSEL & PxSEL2 both registers select the pin attribute
    P1SEL1 &= ~(BIT4+BIT5);        //P1.1 = RXD | P1.2 = TXD

    UCA0CTL1 |= UCSSEL_2;       //SMCLK clock source

    UCA0BRW = UCA0BRW_VAL;       // Setting Baud Rate Control Register
    UCA0MCTLW = UCA0MCTLW_VAL;    //modulation stage comes from baud rate table.
//    UCA0BRW = brDiv;
//    UCA0MCTLW = (secMod<<8);
//    UCA0MCTLW |= (firstMod<<4);
//    UCA0MCTLW |= overSample;


    UCA0CTL0 = 0x00;            //ignore
    UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine

//    IE2 |= UCA0TXIE;                  // Enable the Transmit interrupt
//    _BIS_SR(GIE);                     // Enable the global interrupt
}

void SerialPrintChar(unsigned char c)
{
    UCA0TXBUF = c;
    while(UCA0STATW & UCBUSY);
}

void SerialPrint(char *str)
{
    while(*str)
    {
        UCA0TXBUF = *str;
        while(UCA0STATW & UCBUSY); //UCBUSY Flag = 1(Transmit/Receive) | 0(Inactive)
        str++;
    }
}

void change_uart_set(uint8_t i)
{
    UCA0CTL1 |= UCSWRST;        //USCI logic held in reset state means disabled
    UCA0MCTLW = ((uint16_t)(i<<8) | (UCBRF0_VAL<<4) | UCOS16_VAL);
    UCA0CTL1 &= ~UCSWRST;       // Initialize USCI state machine
}

#endif


void println(void)
{
	SerialPrintChar('\r');
	SerialPrintChar('\n');
}


void SerialPrintln(char *str)
{
	/*SerialPrint(str);
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = '\r';
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = '\n';
	*/
	SerialPrint(str);
	println();
}

char *P(char *str)
{
	return str;
}

void SerialPrintF(const char *str)
{
	SerialPrint(str);
}

void SerialPrintlnF(const char *str)
{
	SerialPrint(str);
	println();
}
/************Hardware Independent Functions************/


void SerialPrintU32(uint32_t n)
{
	char buf[13];
	char *ptr = &buf[sizeof(buf) - 1]; //buf[12] -> last index of buffer
	*ptr = '\0';//null last index

	uint32_t  temp;
	char c;
	do
	{
		temp = n;
		n = n/10;
		c = temp - n*10;
		*--ptr = c + '0';
	}while(n);
	SerialPrint(ptr);
}

void SerialPrintlnU32(uint32_t n)
{
	/*
	char buf[15];
	char *ptr = &buf[sizeof(buf) - 1]; //buf[12] -> last index of buffer
	*ptr = '\0';
	*--ptr  = '\n';
	*--ptr  = '\r';
	uint32_t  temp;
	char c;
	do
	{
		temp = n;
		n = n/10;
		c = temp - n*10;
		*--ptr = c + '0';
	}while(n);
	SerialPrint(ptr);
	*/
	SerialPrintU32(n);
	println();
}

void SerialPrintS32(int32_t n)
{
	if(n < 0)
	{
		SerialPrintChar('-');
		n = -n;
	}
	SerialPrintU32(n);

}

void SerialPrintlnS32(int32_t n)
{
	if(n < 0)
	{
		SerialPrintChar('-');
		n = -n;
	}
	SerialPrintlnU32(n);
}


void SerialPrintU16(uint16_t n)
{
	SerialPrintU32(n);
}

void SerialPrintlnU16(uint16_t n)
{
	SerialPrintlnU32(n);
}

void SerialPrintS16(int16_t n)
{
	SerialPrintS32(n);
}

void SerialPrintlnS16(int16_t n)
{
	SerialPrintlnS32(n);
}

void SerialPrintU8(uint8_t n)
{
	SerialPrintU32(n);
}

void SerialPrintS8(int8_t n)
{
	SerialPrintS32(n);
}


void SerialPrintlnU8(uint8_t n)
{
	SerialPrintlnU32(n);
}

void SerialPrintlnS8(int8_t n)
{
	SerialPrintlnS32(n);
}


void SerialPrintFloat(float n,uint8_t digit)
{
	//Handle negative
	if(n < 0.0)
	{
		SerialPrintChar('-');
		n = -n;
	}
	
	uint32_t integerPart = (uint32_t)n;
	float remainder = n - integerPart;
	SerialPrintU32(integerPart);
	
	//Print decimal point
	if(digit>0)
	{
		SerialPrintChar('.');
	}
	
	//print remainder part
	while(digit-- > 0)
	{
		remainder *= 10.0;
	}
	integerPart = (uint32_t)remainder;
	SerialPrintU32(integerPart);
}

void SerialPrintlnFloat(float n,uint8_t digit)
{
	SerialPrintFloat(n,digit);
	println();
}


// void serial_print_ulong(uint32_t n)
// {
//     char buf[13];
//     char *ptr = &buf[sizeof(buf) - 1]; //buf[12] -> last index of buffer
//     *ptr = '\0';//null last index

//     uint32_t  temp;
//     char c;
//     do
//     {
//         temp = n;
//         n = n/10;
//         c = temp - n*10;
//         *--ptr = c + '0';
//     }while(n);
//     serial_print_str(ptr);
// }

// void serial_print_long(int32_t n)
// {
//     if(n<0)
//     {
//         serial_print_char('-');
//         n = -n;
//     }
//     serial_print_ulong(n);
// }
// void serial_print_uint(uint16_t n)
// {
//     serial_print_ulong(n);
// }

// void serial_print_int(int16_t n)
// {
//     serial_print_long(n);
// }
// void serial_print_uint8(uint8_t n)
// {
//     serial_print_ulong(n);
// }

// void serial_print_int8(int8_t n)
// {

//     serial_print_long(n);
// }

// void serial_debug_print_uint8(uint8_t n,int8_t base)
// {
//     char buf[8*sizeof(uint8_t)+1];
//     char *ptr = &buf[sizeof(buf) - 1];
//     *ptr = '\0';
//     uint8_t  temp;
//     if(base < 1)
//     {
//         base = 10;
//     }
//     char c;
//     do
//     {
//        temp = n;
//        n = n/base;
//        c = temp - n*base;
//        *--ptr = c < 10 ? c + '0' : c + 'A' - 10;
//      }while(n);
//     serial_print_str(ptr);
// }

// void serial_debug_print_int8(int8_t n, int8_t base)
// {
//     if(n<0)
//     {
//       serial_print_char('-');
//       n = -n;
//     }
//     serial_debug_print_uint8(n,base);
// }


// void serialPrint(void *ptr, uint8_t type)
// {
// 	switch(type)
// 	{
// 		case STR:
// 			serial_print_str((char*)ptr);
// 		break;
// 		case INT8:
// 		    serial_print_int8(*((int8_t*)ptr));
// 		break;
// 		case UINT8:
// 		    serial_print_uint8(*((uint8_t*)ptr));
// 		break;
// 		case INT16:
// 		    serial_print_int(*((int16_t*)ptr));
// 		break;
// 		case UINT16:
// 		    serial_print_uint(*((uint16_t*)ptr));
// 		break;
// 		case INT32:
// 			serial_print_long(*((int32_t*)ptr));
// 		break;
// 		case UINT32:
// 			serial_print_ulong(*((uint32_t*)ptr));
// 		break;
// 		case BIN:
// 			serial_debug_print_uint8(*((uint8_t*)ptr),BIN);
// 		break;
// 		case HEX:
// 			serial_debug_print_uint8(*((uint8_t*)ptr),HEX);
// 		break;
// 	}
// }


//void serial_print_int(int var)
//{
//    char buf[10];
//    serial_print(int_to_str(var,buf));
//}
//
//void serial_print_int2(int n)
//{
//    char buf[10];
//    char *ptr = &buf[sizeof(buf)-1];
//    *ptr = '\0';
//    bool neg = false;
//    if(n < 0)
//    {
//        n = -n;
//        neg = true;
//    }
//    do
//    {
//        uint16_t m = n;
//        n = n/10;
//        char c = m-10*n;
//        *--ptr = c+'0';
//    }while(n);
//
//    if(neg)
//    {
//        *--ptr = '-';
//    }
//    serial_print(ptr);
//}

//void set_mcu_clock(void)
//{
//    //Set MCLK = SMCLK = 1MHz
//    BCSCTL1 = CALBC1_1MHZ; // Set DCO Clock to 1MHz
//    DCOCTL = CALDCO_1MHZ;
//}

//#pragma vector = USCIAB0TX_VECTOR
//__interrupt void TransmitInterrupt(void)
//{
//    UCA0TXBUF = *tx_ptr;
//    tx_ptr ++;
//}
