/*
 * msp430g2553_I2C.h
 *	13/07/2020: Updated to receive multiple byte
 *
 *  Created on: Jun 29, 2020
 *      Author: Md Shakil Anwar
 */

#if defined(__MSP430G2553__)
#define SDA BIT7                                                        // i2c sda pin
#define SCL BIT6                                                        // i2c scl pin
#elif defined (__MSP430FR2433__)
#define SDA BIT2                                                        // i2c sda pin
#define SCL BIT3                                                        // i2c scl pin
#endif

#include "mspI2C.h"

#include "../mspDriver.h"

#define TIMEOUT     60000
#define I2C_DELAY   50


int TXByteCtr;					/// Transmit data byte counter
uint8_t *I2C_data_buffer ;	/// Transmit Data Buffer Pointer

#if defined(__MSP430G2553__)

void initiate_I2C_master(char slaveAddress, uint16_t sync_clock_speed) {
	  //P1.6 and P1.7 I2C Data & Clock
      P1SEL    |= SCL + SDA;                                              // Assign I2C pins to USCI_B0
      P1SEL2   |= SCL + SDA;                                              // Assign I2C pins to USCI_B0
      UCB0CTL1 |= UCSWRST;                      // Enable SW reset
      UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
      UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
      if (sync_clock_speed == 0)
      {
          UCB0BR0 = 10;                             // default sync speed fSCL = SMCLK/10 = 100KHz
          UCB0BR1 = 0;
      }else
      {
    	  UCB0BR0 = (unsigned char)(sync_clock_speed & 0xFF);                             // fSCL = SMCLK/sync_clock_speed
    	  UCB0BR1 = (unsigned char)((sync_clock_speed >> 8) & 0xFF);
      }

      UCB0I2CSA = slaveAddress;                         // Slave Address is 069h
      UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
      IE2 |= UCB0RXIE + UCB0TXIE;               //Enable RX and TX interrupt
}

void I2C_transmit_master(int TXByteCounter, char *I2C_dataToSend){
	TXByteCtr = TXByteCounter;
	I2C_data_buffer = I2C_dataToSend;		//copy I2C data address
    while (UCB0CTL1 & UCTXSTP);                                         // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;                                         // I2C TX, start condition
    while (UCB0CTL1 & UCTXSTP);                                         // Ensure start condition got sent
    __bis_SR_register(CPUOFF + GIE);        			// Enter LPM0 w/ interrupts
}

void I2C_receive_master(int TXByteCounter, char *I2C_dataToReceive){
	TXByteCtr = TXByteCounter;
	I2C_data_buffer = I2C_dataToReceive;
    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
    UCB0CTL1 &= ~UCTR ;                     // Clear UCTR
    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
    while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
    UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
}

void I2C_Read_EEProm(unsigned int memory_address, char * data, unsigned char DataLength )
{ //Reading from a 24LCxxx series is much easier then writing.  Reading doesn't have to be done in 64 byte pages.
    /*
         * Todo:
         * 1. add checks to make sure write does not exceed maximum length of EEprom
         * 2. check for valid memory_address
         *
         */
    __disable_interrupt();
    int rLoop = 0;  //loop counter

    while (UCB0STAT & UCBBUSY);         //wait for USCI B0 bus to be inactive

    UCB0CTL1 |= UCTR + UCTXSTT;                     //set USCI to be I2C TX,  send start condition
    UCB0TXBUF = (memory_address & 0x00FF);     //transfer memory_address MSB

    while (UCB0CTL1 & UCTXSTT);                     // waiting for slave address to transfer
    while ((IFG2 & UCB0TXIFG) != UCB0TXIFG);        //wait for TX IFG to clear


    UCB0CTL1 &= ~UCTR;              //set USCI to be RECEIVER
    UCB0CTL1 |= UCTXSTT;            //send restart
    while (UCB0CTL1 & UCTXSTT);     // wait until I2C STT is sent

    for (rLoop=0; rLoop<DataLength; rLoop++)    //receive loop
    {
         while ((IFG2 & UCB0RXIFG) != UCB0RXIFG); //wait for RX buffer to have data
            data[rLoop] = UCB0RXBUF;  //Move rvcd data of or USCI buffer. This also clears the UCB0RXIFG flag

            if (rLoop == DataLength-2)  //NACK and STOP must be send before the last byte is read from the buffer.
                                        //if not the CPU will read an extra byte.
            {
                UCB0CTL1 |= UCTXNACK; //generate a NACK
                UCB0CTL1 |= UCTXSTP;  //generate a stop condition
            }
    }
    _enable_interrupt();

}


#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
                                              // Master Receieve
    *I2C_data_buffer = UCB0RXBUF;             // Get RX data
    __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}

#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{												// Master Receieve
      if (TXByteCtr)                            // Check TX byte counter
      {
    	  UCB0TXBUF = *I2C_data_buffer;           // Load TX buffer
    	  TXByteCtr--;                            // Decrement TX byte counter
    	  I2C_data_buffer++;					  // Increment of buffer address
      }
      else
      {
    	  UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
    	  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
    	  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
       }
}

#elif defined (__MSP430FR2433__)

void i2c_init(void){
    static char init=0;
    if(!init){
        P1REN |= BIT2 | BIT3;
        P1OUT |= BIT2 | BIT3;

        // if the bus is stuck (SDA = low): clock until it recovers
        P1DIR |= BIT3;
        while(!(P1IN & BIT2)){
            P1OUT |= BIT3;
            __delay_cycles(8*80);
            P1OUT &= ~BIT3;
            __delay_cycles(8*80);
        }

        P1SEL0 |= BIT2 | BIT3;                  // I2C pins
        // Configure USCI_B0 for I2C mode
        UCB0CTLW0 |= UCSWRST;                   // Software reset enabled
        UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC; // I2C mode, Master mode, sync
                                                // after UCB0TBCNT is reached
        UCB0BRW = 10;                       // baudrate = SMCLK / 10 = ~100K
        UCB0CTL1 &= ~UCSWRST;
        init = 1;
    }
}

int i2cReadGyroData(char addr,char *data, int len)
{
    unsigned int timeout = 0 ;
    unsigned int i;
    UCB0I2CSA = addr;
    UCB0CTLW0 |= UCTR;
    while (UCB0CTL1 & UCTXSTP)
    {
        __delay_cycles(I2C_DELAY);
        if(timeout++ > TIMEOUT)
        {
            UCB0CTLW0 |= UCTXSTP;
        }
    }
    UCB0CTLW0 |= UCTXSTT;            // I2C TX, start condition


    while(!(UCB0IFG & UCTXIFG0))
    {
       __delay_cycles(I2C_DELAY);
       if(timeout++ > TIMEOUT)
       {
           return 5;
       }

    }

    if(UCB0IFG & UCNACKIFG)
    {
       UCB0CTL1 |= UCTXSTP;               // I2C stop condition
       UCB0IFG = 0;                            // Clear All USCI_B0 flags
       return 2;      // check for NACK
    }

    UCB0TXBUF = data[0];            // Write register address
    timeout = 0;
    while(!(UCB0IFG & UCTXIFG0))
    {
       __delay_cycles(I2C_DELAY);
       if(timeout++ > TIMEOUT)
       {
           return 3;
       }
    }

    UCB0I2CSA = addr;
    UCB0CTLW0 &= ~UCTR;

    UCB0CTLW0 |= UCTXSTT;            // I2C TX, start condition
    while(!(UCB0IFG & UCTXIFG0))
    {
       __delay_cycles(I2C_DELAY);
       if(timeout++ > TIMEOUT)
       {
           return 4;
       }

    }

    if(UCB0IFG & UCNACKIFG)
    {
       UCB0CTL1 |= UCTXSTP;               // I2C stop condition
       UCB0IFG = 0;                            // Clear All USCI_B0 flags
       return 2;      // check for NACK
    }

    i=len;
    int index = 0;
    while(i)
    {
//        data[index] = 0;
        timeout=0;
        i--;
        while(!(UCB0IFG & UCRXIFG0))
        {
            __delay_cycles(I2C_DELAY);
            if(timeout++ > TIMEOUT)
            {
                return -1;
            }
        }
        if( i == 0 )
        {
            UCB0CTLW0 |=UCTXSTP;
        }

        data[index] = UCB0RXBUF;
        index++;
    }

    return 0;

}

int i2cWriteGyroData(char addr,char *data, int len)
{
    unsigned int timeout = 0;
    unsigned int i;
    UCB0I2CSA = addr;
    while (UCB0CTL1 & UCTXSTP)
    {
        __delay_cycles(I2C_DELAY);
        if(timeout++ > TIMEOUT)
        {
            UCB0CTLW0 |= UCTXSTP;
        }
    }
    UCB0CTLW0 |= UCTR | UCTXSTT;            // I2C TX, start condition

    timeout = 0;
    while(!(UCB0IFG & UCTXIFG0))
    {
        __delay_cycles(I2C_DELAY);
        if(timeout++ > TIMEOUT)
        {
            return 5;
        }

    }
    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTL1 |= UCTXSTP;               // I2C stop condition
        UCB0IFG = 0;                            // Clear All USCI_B0 flags
        return 2;      // check for NACK
    }

//    int dataLen = len;
//    i = 0 ;
//    while(dataLen)
//    {
//        timeout=0;
//        dataLen--;
//        UCB0TXBUF = data[i];
//        i++;
//
//        while(!(UCB0IFG & UCTXIFG0))
//        {
//            __delay_cycles(I2C_DELAY);
//            if(timeout++ > TIMEOUT)
//            {
//                return -1;
//            }
//        }
//        if(dataLen == 0)
//        {
//            UCB0CTLW0 |=UCTXSTP;
//            while (UCB0CTL1 & UCTXSTP)
//            {
//                __delay_cycles(I2C_DELAY);
//                if(timeout++ > TIMEOUT)
//                {
//                    return -5;
//                }
//            }
//        }
//
//    }

    for(i=0;i<len;i++){
        UCB0TXBUF = data[i];
        timeout = 0;
        while(!(UCB0IFG & UCTXIFG0))
            __delay_cycles(I2C_DELAY);
            if(timeout++ > TIMEOUT)
                return 3;
    }
    UCB0CTLW0 |=UCTXSTP;
    while (UCB0CTL1 & UCTXSTP)
    {
        __delay_cycles(I2C_DELAY);
        if(timeout++ > TIMEOUT)
        {
            return -5;
        }
    }
    return 0;

}

int i2c_senddata(char *data, int len){
    unsigned int timeout=0;
    unsigned int i;
//    static int dummy=0;
    /*if(dummy){
        UCB0TXBUF = 0;
        timeout = 0;
        while(!(UCB0IFG & UCTXIFG0))
            if(timeout++ > TIMEOUT)
                return -1;
    }
    if(dummy == 0) dummy = 1;
*/
     //__delay_cycles(5000);
    for(i=0;i<len;i++){
        UCB0TXBUF = data[i];
        timeout = 0;
        while(!(UCB0IFG & UCTXIFG0))
            if(timeout++ > TIMEOUT)
                return 3;
    }
    return 0;
}

int i2c_readdata(char *data, int len){
    unsigned int timeout=0;
    unsigned int i;

//    memset(data, 0, len);
    if(len == 1)
    {
        UCB0CTLW0 |= UCTR | UCTXSTP;
    }
    for(i=0;i<len;i++){
        data[i] = 0;
        timeout=0;

        while(!(UCB0IFG & UCRXIFG0))
            if(timeout++ > TIMEOUT)
                return -1;


        data[i] = UCB0RXBUF;
        if(len > 1)
            if(i == len-2)
                UCB0CTLW0 |= UCTR | UCTXSTP;

    }
    //while(!(UCB0IFG & UCTXIFG0));
    return 0;
}

int i2c_start(char addr, int read){
    unsigned int timeout=0;
    UCB0I2CSA = addr;
    if(read)
        UCB0CTLW0 &= ~UCTR;
    else
        UCB0CTLW0 |= UCTR;
    while (UCB0CTL1 & UCTXSTP)
    {
//        __delay_cycles(100);
        if(timeout++ > TIMEOUT)
            UCB0CTLW0 |= UCTXSTP;
    }
    timeout=0;
    UCB0CTLW0 |= UCTXSTT;            // I2C TX, start condition

    while(!(UCB0IFG & UCTXIFG0))
        if(timeout++ > TIMEOUT)
            return 1;

    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTL1 |= UCTXSTP;               // I2C stop condition
        UCB0IFG = 0;                            // Clear All USCI_B0 flags
        return 2;      // check for NACK
    }

    if(!read){
        UCB0TXBUF = 0;
        while(!(UCB0IFG & UCTXIFG0));
    }

    return 0;
}

int i2c_start2(char addr, int read){
    unsigned int timeout=0;
    UCB0I2CSA = addr;
    if(read)
        UCB0CTLW0 &= ~UCTR;
    else
        UCB0CTLW0 |= UCTR;
    while (UCB0CTL1 & UCTXSTP);
    UCB0CTLW0 |= UCTXSTT;            // I2C TX, start condition

    while(!(UCB0IFG & UCTXIFG0))
    {
        __delay_cycles(50);
        if(timeout++ > TIMEOUT)
            return 5;
    }

    if(UCB0IFG & UCNACKIFG)
    {
        UCB0CTL1 |= UCTXSTP;               // I2C start condition
        UCB0IFG = 0;                            // Clear All USCI_B0 flags
        return 6;      // check for NACK
    }

    /*if(!read){
        UCB0TXBUF = d;
        while(!(UCB0IFG & UCTXIFG0));
    }*/

    return 0;
}

int i2c_stop(void){
    unsigned int timeout=0;
    UCB0CTLW0 |= UCTR | UCTXSTP;            // I2C TX, stop condition
    while(UCB0IFG & UCTXSTP)
        if(timeout++ > TIMEOUT)
            return 4;
    return 0;
}





//
//bool initiate_I2C_master(uint8_t slaveAddress, uint16_t sync_clock_speed) {
////      UCB0IE &= ~(UCRXIE + UCTXIE);
//      UCB0CTLW0 |= UCSWRST;                      // Enable SW reset
//    //P1.2 and P1.3 I2C Data & Clock
//      P1SEL0   |= SCL + SDA;                                              // Assign I2C pins to USCI_B0
//      P1SEL1   &= ~(SCL + SDA);                                              // Assign I2C pins to USCI_B0
////      ADCPCTL2  = 0x00;                                                        // disable A2 ADC
////      ADCPCTL3  = 0x00;                                                        // disable A3 ADC
//
//      UCB0CTLW0 = UCMST     |                   // I2C Master Mode
//                  UCMODE_3  |                   // I2C Mode
//                  UCSYNC    |                   // Synchronous Mode
//                  UCSSEL_2  |                   // Select SMCLK as clock source
//                  UCSWRST;                      // Soft Reset
//
//      if (sync_clock_speed == 0)
//      {
//          UCB0BRW = 10;                             // default sync speed fSCL = SMCLK/10 = 100KHz
//      }else
//      {
//          UCB0BRW = sync_clock_speed;                             // fSCL = SMCLK/sync_clock_speed
//      }
//
//      UCB0I2CSA = slaveAddress;                         // Slave Address is 069h
//      UCB0CTLW0 &= ~UCSWRST;                     // Clear SW reset, resume operation
////      UCB0IE |= UCRXIE + UCTXIE;               //Enable RX and TX interrupt
//
//
//      // Checking I2C address is valid or not
//      while (UCB0CTL1 & UCTXSTP);                                         // Ensure stop condition got sent
////      SerialPrint("\r\nstop condition got sent.");
//      UCB0CTL1 |= UCTR + UCTXSTT;                                         // I2C TX, start condition
//      while (UCB0CTL1 & UCTXSTP);                                         // Ensure start condition is sent
////      SerialPrint("\r\nstart condition is sent.");
//
//      uint32_t tempT = millis();
//
//      while(!(UCB0IFG & UCTXIFG0))
//      {
//          SerialPrint("\r\n I2C Address is not matched yet.");
//          if(UCB0IFG & UCNACKIFG)
//          {
//              UCB0CTL1 |= UCTXSTP;               // I2C start condition
//              UCB0IFG = 0;                            // Clear All USCI_B0 flags
//              return false;      // check for NACK
//          }
//
//          if((millis() - tempT) > MSP_I2C_TIMEOUT)
//          {
//              break;
//          }
//      }
//      if(UCB0IFG & UCTXIFG0){
//          return true;
//      }else
//      {
//          return false;
//      }
//
//}
//
//
//uint8_t I2C_transmit_master(uint8_t *I2C_dataToSend, int TXByteCounter){
//    int i=0;
//    TXByteCtr = TXByteCounter;
//
////    SerialPrint("\r\nTMP117 reg addr: ");
////    SerialPrintU8(*I2C_dataToSend);
//
//    I2C_data_buffer = I2C_dataToSend;       //copy I2C data address
//    UCB0CTL1 |= UCTXSTP;
//    while (UCB0CTL1 & UCTXSTP);                                         // Ensure stop condition got sent
//    UCB0CTL1 |= UCTR + UCTXSTT;                                         // I2C TX, start condition
////    while (UCB0CTL1 & UCTXSTT);                                         // Ensure start condition is sent
//
//    uint32_t tempT = millis();
//
//    while(!(UCB0IFG & UCTXIFG0))
//    {
//      if((millis() - tempT) > MSP_I2C_TIMEOUT)
//      {
//        return 5;
//      }
//    }
//    for(i=0; i<TXByteCtr; i++){
//        SerialPrintU8(*I2C_data_buffer);
//        UCB0TXBUF = *I2C_data_buffer;
////        SerialPrint("\r\n status register ");
////        SerialPrintU16(UCB0IFG);
//        tempT = millis();
//        while(!(UCB0IFG & UCTXIFG0)){
//            if((millis() - tempT) > MSP_I2C_TIMEOUT)
//            {
//
//                return 3;
//            }
//        }
//        I2C_data_buffer++;
//    }
//    UCB0CTLW0 |= UCTR + UCTXSTP;            // I2C TX, stop condition
//    tempT = millis();
//    while(UCB0IFG & UCTXSTP)
//    {
//        if((millis() - tempT) > MSP_I2C_TIMEOUT)
//        {
//            return 4;
//        }
//    }
//
//    if(UCB0IFG & UCNACKIFG)
//    {
//        UCB0CTL1 |= UCTXSTP;               // I2C start condition
//        UCB0IFG = 0;                            // Clear All USCI_B0 flags
//        return 6;      // check for NACK
//    }
//    return 0;
//
//}
//
//uint8_t I2C_receive_master(uint8_t *I2C_dataToReceive, int TXByteCounter){
//    int i = 0;
//    TXByteCtr = TXByteCounter;
//    I2C_data_buffer = I2C_dataToReceive;
//    UCB0CTL1 |= UCTXSTP;
//    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
//    UCB0CTL1 &= ~UCTR ;                     // Clear UCTR
//    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
////    while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
//
//    uint32_t tempT = millis();
//
//    while(!(UCB0IFG & UCTXIFG0))
//    {
//      if((millis() - tempT) > MSP_I2C_TIMEOUT)
//      {
//        return 5;
//      }
//    }
//
//    if(TXByteCtr > 1) {
//        for(i=0;i<TXByteCtr;i++){
//            I2C_data_buffer[i] = 0;
//            tempT = millis();
//            while(!(UCB0IFG & UCRXIFG0)){
//                if((millis() - tempT) > MSP_I2C_TIMEOUT)
//                {
//                    return -1;
//                }
//            }
//
//            I2C_data_buffer[i] = UCB0RXBUF;
//            if(i == (TXByteCtr-2))
//            {
//                UCB0CTLW0 |= UCTXSTP;
//            }
//        }
//    }
//
//    tempT = millis();
//    while(UCB0IFG & UCTXSTP)
//    {
//        if((millis() - tempT) > MSP_I2C_TIMEOUT)
//        {
//            return 4;
//        }
//    }
//
//
//    if(UCB0IFG & UCNACKIFG)
//    {
//        UCB0CTL1 |= UCTXSTP;               // I2C start condition
//        UCB0IFG = 0;                            // Clear All USCI_B0 flags
//        return 6;      // check for NACK
//    }
//
//    return 0;
//
//}
//
//void I2C_set_slave_addr(uint8_t slaveAddress)
//{
//    UCB0I2CSA = slaveAddress;
//}



//void I2C_transmit_master(char *I2C_dataToSend, int TXByteCounter){
//    TXByteCtr = TXByteCounter;
//    I2C_data_buffer = I2C_dataToSend;       //copy I2C data address
//    while (UCB0CTL1 & UCTXSTP);                                         // Ensure stop condition got sent
//    UCB0CTL1 |= UCTR + UCTXSTT;                                         // I2C TX, start condition
//    while (UCB0CTL1 & UCTXSTP);                                         // Ensure start condition is sent
////    __bis_SR_register(CPUOFF + GIE);                    // Enter LPM0 w/ interrupts
//}
//
//void I2C_receive_master(char *I2C_dataToReceive, int TXByteCounter){
//    TXByteCtr = TXByteCounter;
//    I2C_data_buffer = I2C_dataToReceive;
//    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent
//    UCB0CTL1 &= ~UCTR ;                     // Clear UCTR
//    UCB0CTL1 |= UCTXSTT;                    // I2C start condition
//    while (UCB0CTL1 & UCTXSTT);             // Start condition sent?
//    if (TXByteCtr == 1)                            // Check TX byte counter
//    {
//        UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
//    }
//    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
//}
//
//
//
//
//#pragma vector = USCI_B0_VECTOR
//__interrupt void USCI_B0_ISR(void) {
//    switch(__even_in_range(UCB0IV,0x1e)) {
//        case 0x00: // Vector 0: No interrupts
//            break;
//        case 0x0a:  // Vector 10: RXIFG3
//            break;
//        case 0x0c:  // Vector 12: TXIFG3
//            break;
//        case 0x0e:  // Vector 14: RXIFG2
//            break;
//        case 0x10:  // Vector 16: TXIFG2
//            break;
//        case 0x12:  // Vector 18: RXIFG1
//            break;
//        case 0x14:  // Vector 20: TXIFG1
//            break;
//        case 0x16:  // Vector 22: RXIFG0
//            USCIAB0RX_ISR();
//            break;
//        case 0x18:  // Vector 24: TXIFG0
//            USCIAB0TX_ISR();
//            break;
//        default: break;
//    }
//}
//
//void USCIAB0TX_ISR(void)
//{                                               // Master Receieve
//      if (TXByteCtr)                            // Check TX byte counter
//      {
//          UCB0TXBUF = *I2C_data_buffer;           // Load TX buffer
//          TXByteCtr--;                            // Decrement TX byte counter
//          I2C_data_buffer++;                      // Increment of buffer address
//      }
//      else
//      {
//          UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
//          UCB0IFG &= ~UCTXIFG0;                     // UCTXIFG0 int flag is automatically cleared in fr2433
//          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
//       }
//}
//
//void USCIAB0RX_ISR(void)
//{
//    if(TXByteCtr>1)
//    {
//        // Master Receieve
//        *I2C_data_buffer = UCB0RXBUF;             // Get RX data
//        TXByteCtr--;                            // Decrement TX byte counter
//        I2C_data_buffer++;                      // Increment of buffer address
//    }else
//    {
//        UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
//        UCB0TXBUF = *I2C_data_buffer;           // Load TX buffer
//        __bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
//    }
//}

#endif
