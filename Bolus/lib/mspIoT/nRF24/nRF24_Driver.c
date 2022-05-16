/*
   NRF24_Driver.c
   Created on: Aug 27, 2020
   Author: Shuvangkar & Shakil Anwar
   Description: The hardware layer abstraction for interfacing nRf24L RF module 
*/
#include "nRF24_Driver.h"


volatile uint8_t _nrfStatusReg;
volatile bool _nrfDebug;
/*********GPIO variable***********/
 volatile gpio_pin *_cePort;
 volatile gpio_pin _cePin;
 volatile gpio_pin *_csnPort;
 volatile gpio_pin _csnPin;

/***********************Driver Basic API************************/
//Read a single byte from nrf24l at particular addr and return the value 
uint8_t read_register(uint8_t addr)
{
  uint16_t response;

  nrf_csn_low();
  response = spi_transfer16(RF24_NOP | ((addr & RF24_REGISTER_MASK) << 8));
  nrf_csn_high();

  _nrfStatusReg = (uint8_t) ((response & 0xFF00) >> 8);
  return (uint8_t) (response & 0x00FF);
}

//write single byte to the particular addr
void write_register(uint8_t addr, uint8_t data)
{
  uint16_t response;

  nrf_csn_low();
  response = spi_transfer16( (data & 0x00FF) | (((addr & RF24_REGISTER_MASK) | RF24_W_REGISTER) << 8) );
  nrf_csn_high();
  _nrfStatusReg = (uint8_t) ((response & 0xFF00) >> 8);
}

//write few bytes in a particular register
void write_bytes_in_register(uint8_t addr, uint8_t *payload, uint8_t len)
{
  nrf_ce_low();
  uint8_t *ptr = payload;

  nrf_csn_low();
  _nrfStatusReg = spi_transfer(addr | RF24_W_REGISTER);
  uint8_t i;
  for ( i = 0; i < len ; i++)
  {
    spi_transfer(ptr[i]);
  }
  nrf_csn_high();
}

//read few bytes from particular register
uint8_t *read_bytes_in_register(uint8_t addr, uint8_t *bucket, uint8_t len)
{
  uint8_t *ptr = bucket;

  nrf_csn_low();
  _nrfStatusReg = spi_transfer(addr & RF24_REGISTER_MASK);
  uint8_t i;
  for ( i = 0; i < len; i++)
  {
    ptr[i] = spi_transfer(0xFF);
  }
  nrf_csn_high();

  return bucket;
}

//This function is very useful when you want to set a particular bit in a register 
//without changing the whole register. 
void set_reg_bit(uint8_t reg, uint8_t bitMask)
{
	uint8_t regValue = read_register(reg);
	write_register(reg,regValue | bitMask);
}

//This function is very useful when you want to clear a particular bit in a register 
//without changing the whole register. 
void clear_reg_bit(uint8_t reg, uint8_t bitMask)
{
	uint8_t regValue = read_register(reg);
	write_register(reg, regValue & ~bitMask);
}

//Clear nrf tx all buffer memory 
void nrf_flush_tx()
{
  nrf_csn_low();
  _nrfStatusReg = spi_transfer(RF24_FLUSH_TX);
  nrf_csn_high();

}

//clear nrf rx buffer memory 
void nrf_flush_rx()
{
  nrf_csn_low();
  _nrfStatusReg = spi_transfer(RF24_FLUSH_RX);
  nrf_csn_high();
}

/****************************Driver Extended API*********************/

//This function sets the channel radio frequency. 
void nrfSetRfChannel(uint8_t ch)
{
  // set nrf channel
  //If channel exceeds rset value to 100 channel.
  if (ch > 125)
  {
    ch = 100;
  }
  write_register(RF24_RF_CH, (ch & 0b01111111));  
}

//this function write the TX address. 
void nrf_set_tx_addr(uint8_t *addr, uint8_t len)
{
  write_bytes_in_register(RF24_TX_ADDR, addr, len);
}

//Write RX address in a particular pipe. For pipe 0-1 sets 5 bytes and for rest 
// of the pipes set on LS byte 
void nrf_set_rx_addr(uint8_t pipe, uint8_t *addr, uint8_t len)
{

  if (pipe > 5)
  {
    pipe = 1;
  }

  if (pipe > 1)
  {
    write_register(RF24_RX_ADDR_P0 + pipe, addr[0]);
  } 
  else 
  {
    write_bytes_in_register(RF24_RX_ADDR_P0 + pipe, addr, len);
  }

  //RX Address PIPE Enable
  set_reg_bit(RF24_EN_RXADDR, (1 << pipe));
}

//Set nrf TX and RX address width for the system 
void nrf_set_addr_width(uint8_t width)
{
  // if (width < 3 || width > 5)
  // {
  //   return;
  // }
  write_register(RF24_SETUP_AW, (width - 2) & 0b00000011);
}

//Set RF power and Data transfer speed 
void nrf_set_tx_dbm_speed(uint8_t power_speed)
{
  write_register(RF24_RF_SETUP, (power_speed) & 0x2F);
}

//
void dlp_enable()
{
  write_register (RF24_FEATURE,EN_DLP);
}


void dlp_disable()
{
  clear_reg_bit(RF24_FEATURE,EN_DLP);
}

//dynamic paylod disable 
void dynpd_disable(uint8_t pipe)
{
	clear_reg_bit(RF24_DYNPD,1<<pipe);

}

//dynamic payload enable 
void dynpd_enable(uint8_t pipe)
{
	set_reg_bit(RF24_DYNPD,1<<pipe);
}

//nrf serial print any buffer 
void nrfPrintBuffer(void *ptr, uint8_t len)
{
  uint8_t i;
  uint8_t *p = ptr;
  for(i = 0; i<len; i++)
  {
    SerialPrintU8(p[i]);
    SerialPrintF(P("  ")); 
    //SerialPrintChar('  ');
  }
  //SerialPrintChar('\n');
  SerialPrintF(P("\r\n")); 
}


/*****************************Generic function**********************/

//calculate checksum of the buffer and return 8 bit result 
uint8_t checksum(void *buffer, uint8_t len)
{
  uint8_t *p = (uint8_t*)buffer;
  
  uint16_t sum = 0;
  uint8_t i;
  for(i = 0; i< len; i++)
  {
    sum += (uint16_t)p[i];
  }
  return (uint8_t)sum;
}
