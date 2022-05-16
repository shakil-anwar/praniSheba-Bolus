/*
 * SPI.c
 *
 *  Created on: Aug 26, 2020
 *      Author: Shuvangkar
 */

#include "spi_driver.h"

//#define SPI_CS P3.6

#if defined(__MSP430G2553__)


// void spi_begin(void)
void spi_begin(uint32_t speed)
{
    UCB0CTL1 |= UCSWRST ;               //SPI is on hold for configuration

    P1SEL   |= BIT5 | BIT6 | BIT7 ;     //P1.5 = SCK | P1.6 = MISO | P1.7 = MOSI
    P1SEL2  |= BIT5 | BIT6 | BIT7;

//    delay(10);
    UCB0CTL1 |= UCSSEL_2 + UCSWRST;     //SPI Clock = SMCLK
    UCB0BR0 = 0x01;                     // SPI clocked at same speed as SMCLK
    UCB0BR1 = 0x00;
//    UCB0CTL0 |= UCMODE_0; //3 pin SPI
//    UCB0CTL0 |= UCSYNC; //Synchronous mode
//    UCB0CTL0 |= UCMST; //Master Mode
//    UCB0CTL0 |= UCCKPH; //Sampled first edge| shift on next edge
    UCB0CTL0 |= UCMODE_0 |
                  UCSYNC |
                   UCMST |
                   UCMSB |
                   UCCKPH;

    UCB0CTL1 &= ~UCSWRST; //Start SPI state machine.
}

uint8_t spi_transfer(uint8_t reg)
{
    UCB0TXBUF = reg;
    while (!(IFG2 & UCB0RXIFG));

    return UCB0RXBUF;
}
uint16_t spi_transfer16(uint16_t input16)
{
    uint16_t ret16;
    uint8_t *ret8 = (uint8_t*)&ret16;
    uint8_t *input8 = (uint8_t*)&input16;

    UCB0TXBUF = input8[1];
    while (!(IFG2 & UCB0RXIFG));
    ret8[1] = UCB0RXBUF; //Status

    UCB0TXBUF = input8[0];
    while (!(IFG2 & UCB0RXIFG));
    ret8[0] = UCB0RXBUF; //data
    return ret16;

}

#elif defined (__MSP430FR2433__)

void spi_begin(uint32_t speed)
{

//    UCB0CTLW0 |= UCSWRST ;               //SPI is on hold for configuration

    P2SEL0  |=   BIT4 | BIT5 | BIT6 ;     //P2.4 = SCK | P2.5 = MISO | P2.6 = MOSI

    UCA1CTLW0 |= UCSWRST; // **Put state machine in reset**
    UCA1CTLW0 |= UCMST|UCSYNC|UCCKPH|UCMSB|UCMODE_0; // 3-pin, 8-bit SPI master
    // Clock polarity high, MSB
    UCA1CTLW0 |= UCSSEL__SMCLK; // SMCLK
    UCA1BR0 = 0x09; // /2,fBitClock = fBRCLK/(UCBRx+1).
    UCA1BR1 = 0; //
    UCA1MCTLW = 0; // No modulation
    UCA1CTLW0 &= ~UCSWRST; // **Initialize USCI state machine**
}

uint8_t spi_transfer(uint8_t reg)
{
    UCA1TXBUF = reg;
    while (!(UCA1IFG & UCTXIFG)); // USCI_A1 TX buffer ready?
    while (!(UCA1IFG & UCRXIFG)); // USCI_A1 RX flag set?

    return UCA1RXBUF;
}
uint16_t spi_transfer16(uint16_t input16)
{
    uint16_t ret16;
    uint8_t *ret8 = (uint8_t*)&ret16;
    uint8_t *input8 = (uint8_t*)&input16;

    UCA1TXBUF = input8[1];
    while (!(UCA1IFG & UCTXIFG)); // USCI_A1 TX buffer ready?
    while (!(UCA1IFG & UCRXIFG)); // USCI_A1 RX flag set?
    ret8[1] = UCA1RXBUF; //Status

    UCA1TXBUF = input8[0];
    while (!(UCA1IFG & UCTXIFG)); // USCI_A1 TX buffer ready?
    while (!(UCA1IFG & UCRXIFG)); // USCI_A1 RX flag set?
    ret8[0] = UCA1RXBUF; //data
    return ret16;
}


#endif
