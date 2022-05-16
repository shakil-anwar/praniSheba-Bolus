/*
   nrf24.c
   Created on: Aug 31, 2020
   Author: Shuvangkar Chandra Das & Shakil Anwar
   Description :  This is the nrf base api for creating user codebase. 
*/

#include "nRF24.h"

#define nrfRXMode()     set_reg_bit(RF24_CONFIG,RX_MODE)
#define nrfTXMode()     clear_reg_bit(RF24_CONFIG,RX_MODE)
#define nrfClearTxDs() write_register(RF24_STATUS, TX_DS)
#define nrfClearRxDr() write_register(RF24_STATUS, RX_DR)
#define nrfClearMaxRt() write_register(RF24_STATUS, MAX_RT)

/**********local function declaration**************/
void nrfEnableAck(uint8_t pipe); // added pipe no as parameter, pipe 6 means set autoack on all pipes
void nrfSetPldAck();
void nrfClearIrq();
void nrfReUseTX();
void nrfReset();

/*********global variable declaration**************/
volatile enum nrf_irq_state_t _nrfIrqState;
volatile uint32_t (*_Millis)(void) = NULL;
volatile uint32_t (*_seconds)(void) = NULL;

static isrPtr_t tx_isr;
static isrPtr_t rx_isr;
static isrPtr_t max_rt_isr;


//This function sets millis() and second() function reference for 
// proper operation. millis is mandatory second is optional. 
void nrfSetTimers(uint32_t (*ms)(), uint32_t (*second)())
{
  _Millis = ms;
  _seconds = second;
}

//This function set nrf ce and csn pin in port and pin format.
//easy to set for microcontroller project 
void nrfSetPin(volatile gpio_pin *cePort,volatile gpio_pin cePin,
              volatile gpio_pin *csnPort,volatile gpio_pin csnPin)
{
#if defined(__MSP430FR2433__)
//  cePin = cePin;
  csnPin = csnPin +8;
#endif
  _cePort = cePort;
  _cePin = cePin;
  _csnPort = csnPort;
  _csnPin = csnPin;

#if defined(ARDUINO_ARCH_AVR)
  cePort--;
  csnPort--;
#elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
  cePort++;
  csnPort++;
#endif

  *cePort |= 1<<cePin;// setting ddr register
  *csnPort |= 1<<csnPin; //setting ddr register
   
   *_cePort &= ~(1 << _cePin); //radio disable
   *_csnPort |=  (1<< _csnPin); //chip disable 
   
}

//reset registers. so that it can be configured from beginning 
void nrfReset()
{
    delay(100); //Power on reset 100ms
    // Disable nrf Interrupts, set CRC 2 Bytes, POWER DOWN, TX MODE
    write_register(RF24_CONFIG, DIS_ALL_INT | CRC_EN_2_BYTE | PWR_DOWN | TX_MODE);
    // clear Interrupt flag
    write_register(RF24_STATUS, TX_DS | MAX_RT | RX_DR);
    // disable all pipes
    write_register(RF24_EN_RXADDR, 0x00);
    // disable auto acknowledgement in all pipes
    write_register(RF24_EN_AA, 0x00);
    // disable dynamic payload in all pipes
    write_register(RF24_DYNPD, 0x00);
}

//This function is the begin function for nrf24l01
void nrfBegin(air_speed_t speed, power_t power, uint32_t spiSpeed)
{
  _nrfIrqState = NRF_WAIT;
  // initiate SPI
  spi_begin(spiSpeed);
  nrf_csn_low();
  spi_transfer(RF24_NOP);
  nrf_csn_high();
  // set default channel
  nrfSetRfChannel(RF_CHANNALE_NO);
  // set payload len
  nrf_set_addr_width(DEFAULT_ADDR_LEN);
  // set retransmit count and retransmit delay
  write_register(RF24_SETUP_RETR, RETRANSMIT_OPCODE | RETRANSMIT_COUNT);
  // set transmit power and data speed
  nrf_set_tx_dbm_speed(((uint8_t)speed | (uint8_t)power));
  // flush rx
  nrf_flush_tx();
  // flush tx
  nrf_flush_rx();
  // enable interrupt
  write_register(RF24_CONFIG, EN_ALL_INT | CRC_EN_2_BYTE | PWR_DOWN |TX_MODE);
}


//nrf enable acknowledgement in a particular pipe 
void nrfEnableAck(uint8_t pipe)
{
  // enable dynamic payload length in feature
  dlp_enable();
  // enable dynamic payload on pipe
  dynpd_enable(pipe);
  //Enable Auto acknowledgment on pipe
  set_reg_bit(RF24_EN_AA, (1 << pipe));
}


//nrf to standby mode1. this functions is used before changing mode. 
//From rx to tx or tx to rx 
void nrfStandby1()
{
   set_reg_bit(RF24_CONFIG, PWR_UP);
   delay(2);
}

//This function turn radio and power down chip optjmize power
void nrfPowerDown()
{
  clear_reg_bit(RF24_CONFIG, PWR_UP);
}

/***********************************************************************************
 *                                         RX Functions
 * *********************************************************************************/   

//Set pipe address and acknowledgment option 
void nrfSetPipe(uint8_t pipe, uint8_t *addr, bool ackFlag)
{
   nrf_ce_low(); // if device is power up, this ensures standby-1
  // set pipe addr
  nrf_set_rx_addr(pipe, addr, DEFAULT_ADDR_LEN);

  if (ackFlag == 1)
  {
    nrfEnableAck(pipe);
  }
}

//start in rx mode 
void nrfRxStart()
{
  nrf_ce_low();
  nrfClearIrq();
  nrf_flush_rx();
  nrfRXMode();
  nrf_ce_high();
  delay_us(150);
}

/***********************************************************************************
 *                                         TX Functions
 * *********************************************************************************/

//configure nrf for tx mode and set address 
void nrfTxBegin(uint8_t *addr, bool ackFlag)
{
  nrf_ce_low();// if device is power up, this ensures standby-1
  // set TX address
  write_bytes_in_register(RF24_TX_ADDR, addr, DEFAULT_ADDR_LEN);
  
  if (ackFlag == 1)
  {
  // set address in pipe 0
    nrf_set_rx_addr(0,addr,DEFAULT_ADDR_LEN);
  // enable auto ack on pipe 0
    nrfEnableAck(0);
  }
}

//tx start operation after calling  this function 
void nrfTxStart()
{
  nrf_ce_low();
  nrfClearIrq();
  nrf_flush_tx();
  nrfTXMode();
  nrf_ce_low(); //NRF goes to Standby -1. 
  // delay_us(130);
}

//When data in tx buffer and calling this function start transmission 
void nrtTxStartTransmission()
{
  nrf_ce_high(); 
  delayMicroseconds(15);
  nrf_ce_low();
}



/***********************************************************************************
 *                                         NRF Status Functions
 * *********************************************************************************/
//This function check whether nrf is responding or not. It has to call after nrfBegin()
//Otherwise the functions will not work 
bool nrfIsRunning()
{
  uint8_t awReg = read_register(RF24_SETUP_AW);
  // SerialPrintF(P("Config :"));SerialPrintlnU8(awReg);
  return ((awReg >= 1) && (awReg <=3));
}

//check nrf Rx FIFI buffer is full. 
bool nrfRxFifiFull()
{
  // check RXFULL pin RF24_FIFO_STATUS register
  if(read_register(RF24_FIFO_STATUS) & RX_FULL) return true;
  else return false;
  
}

//read status register 
uint8_t nrfReadStatus()
{
  // send RF24_NOP return output
  nrf_csn_low();
  uint8_t reg = spi_transfer(RF24_NOP);
  nrf_csn_high();
  return reg;
}

//read nrf rx received payload length 
uint8_t nrfPayloadLen()
{
  uint16_t response;
  nrf_csn_low();
  response = spi_transfer16((RF24_R_RX_PL_WID << 8) | RF24_NOP);
  nrf_csn_high();
  return (uint8_t)(response);
}

//check any pending inetrrupt status bit 
bool nrfIntStatus(uint8_t bitPos)
{
  nrf_csn_low();
  uint8_t reg = spi_transfer(RF24_NOP);
  nrf_csn_high();
  return ((reg >>bitPos) & 0x01);
}

//check payload available in which pipe 
uint8_t pipeAvailable()
{
  // check pipe no in RF24_STATUS register
  nrf_csn_low();
  uint8_t reg = spi_transfer(RF24_NOP);
  nrf_csn_high();
  return ((reg>>1) & PIPE_MASK);
}

//fast  check payload available in which pipe 
uint8_t pipeAvailFast()
{
  //This functions return pipe availableif _nrfStatusReg is done reading already
  return ((_nrfStatusReg>>1) & PIPE_MASK);
}

//check whether rx payload is empty or not. 
bool nrfIsRxEmpty()
{
  //returns true if RX FIFO empty
  // uint8_t reg = read_register(RF24_FIFO_STATUS);
  return (read_register(RF24_FIFO_STATUS) & RX_EMPTY);
}

//check is tx fifi buffer is empty or not 
bool nrfTxIsFifoEmpty()
{
  return (read_register(RF24_FIFO_STATUS) & TX_EMPTY);
}


//print nrf working mode
void nrfPrintMode(enum nrf_mode_t mode)
{
  SerialPrintF(P("<NRF_ "));
  switch (mode)
  {
    case UNDEFINED:
      SerialPrintlnF(P("UNDEF>"));
    break;
    case POWER_DOWN:
      SerialPrintlnF(P("PWDN>"));
    break;
    case STANDBY1:
       SerialPrintlnF(P("STBY1>"));
    break;
    case STANDBY2:
      SerialPrintlnF(P("STBY2>"));
    break;
    case RXMODE:
      SerialPrintlnF(P("RX>"));
    break;
    case TXMODE:
      SerialPrintlnF(P("TX>"));
    break;
  }
}

//determine nrf operating in which mode
enum nrf_mode_t nrfWhichMode()
{
  uint8_t config = read_register(RF24_CONFIG);

  enum nrf_mode_t nrfMode = UNDEFINED;
  if(config & PWR_UP)
  {
    //Chip power up
    uint8_t cePin = nrf_ce_read();
    // if(_nrfDebug){SerialPrintF(P("CE_PIN : "));SerialPrintlnU8(cePin && true);}

    if(config & RX_MODE)
    {
      if(cePin)
      {
        nrfMode = RXMODE;
      }
    }
    else
    {
      if(cePin)
      {
        if(nrfTxIsFifoEmpty())
        {
          nrfMode = STANDBY2;
        }
        else
        {
          nrfMode = TXMODE;
        }
        
      }
      else
      {
        nrfMode = STANDBY1;
      }
    }
  }
  else
  {
    //chip power down
    nrfMode = POWER_DOWN;
  }

  if(_nrfDebug) {nrfPrintMode(nrfMode);}
  return nrfMode;
}

/***********************************************************************************
 *                                         NRF Basic Read/Write Functions
 * *********************************************************************************/

//write payload to transmit 
void nrfWrite(const uint8_t *data, uint8_t len)
{
  nrf_ce_low();//Goes to Standby-1 before writing payload.
  // write_bytes_in_register(RF24_W_TX_PAYLOAD, data, len);
  // nrf_flush_tx();
  nrf_csn_low();
  uint8_t i;
  spi_transfer(RF24_W_TX_PAYLOAD);
  for(i = 0; i<len; i++)
  {
    spi_transfer(data[i]);
  }
  nrf_csn_high();
  _nrfIrqState = NRF_WAIT;

}

//write and send payload 
void nrfSend(const uint8_t *data, uint8_t len)
{
  nrfWrite(data,len);
  nrtTxStartTransmission();
}

//change nrf mode to tx and send payload 
void nrfToTxAndSend(uint8_t *addr,const uint8_t *data, uint8_t len)
{
  //This function set NRF to TX mode send data
  nrfTxBegin(addr, true);
  nrfTxStart();
  nrfWrite(data, len); //NRF_WAIT state
  nrtTxStartTransmission();
}

//nrf acknowledge wait function after data send 
bool nrfAck()
{
  uint32_t currentMs = _Millis();
  uint32_t prevMs = currentMs;
  do
  {
    if(_nrfIrqState == NRF_SUCCESS)
    {
      return true;
    }
    else if(_nrfIrqState == NRF_FAIL)
    {
      break;
    }
    currentMs = _Millis();
  }while((currentMs - prevMs) < NRF_TRANSMIT_TIMEOUT);
  return false; 
}

//nrf send data and return acknowledge 
bool nrfAckSend(const uint8_t *data, uint8_t len)
{
  nrfWrite(data,len);
  nrtTxStartTransmission();
  return nrfAck();
}

//nrf read payload in rx mode 
uint8_t *nrfRead(uint8_t *buffer, uint8_t len)
{
  uint8_t *ptr = buffer;
  nrf_csn_low();
  uint8_t i;
  spi_transfer(RF24_R_RX_PAYLOAD);
  for(i = 0; i<len; i++)
  {
    ptr[i] = spi_transfer(RF24_NOP);
  }
  nrf_csn_high();
  // nrf_flush_rx();
  return buffer;
}



/***********************************************************************************
 *                                         NRF IRQs
 * *********************************************************************************/

//nrf set irq functions
void nrfSetIrqs(isrPtr_t txIsr, isrPtr_t rxIsr, isrPtr_t maxRtIsr)
{
  tx_isr = txIsr;
  rx_isr = rxIsr;
  max_rt_isr = maxRtIsr;
}

//nrf irq handler function
void nrfIrq()
{
  nrfClearIrq(); //This function clear IRQ bits and read the status in _nrfStatusReg
  // _nrfStatusReg = nrfReadStatus();
  // if(_nrfDebug){SerialPrintF(P("NRF STATUS:"));SerialPrintlnU8(_nrfStatusReg);}
  if(_nrfStatusReg & TX_DS)
  {
    _nrfIrqState = NRF_SUCCESS;
    if(tx_isr)
    {
      tx_isr();
    }
    // _nrfIrqState = NRF_SUCCESS;
    if(_nrfDebug){SerialPrintlnF(P("<tx_irq>"));}
  }
  else if(_nrfStatusReg & RX_DR)
  {
    _nrfIrqState = NRF_RECEIVED;
    if(rx_isr)
    {
      rx_isr();
    }
    // _nrfIrqState = NRF_RECEIVED;
    if(_nrfDebug){SerialPrintlnF(P("<rx_irq>"));}
  }
  else
  {
    _nrfIrqState = NRF_FAIL;
    if(max_rt_isr)
    {
      max_rt_isr();
    }
    // _nrfIrqState = NRF_FAIL;
    if(_nrfDebug){SerialPrintlnF(P("<max_rt_irq>"));}
  }
}

//return irq state 
enum nrf_irq_state_t waitAck()
{
  return _nrfIrqState;
}

/***********************************************************************************
 *                                         NRF Utility
 * *********************************************************************************/
void nrfSetPldAck()
{
  // set acknowledgment payload ack FEATURE register
  set_reg_bit(RF24_FEATURE,EN_ACK_PAY);
}

//Clear All type interrupt request flags
void nrfClearIrq()
{
  // clear all irq in flag = 0
  // or clear specific flag in status register
  write_register(RF24_STATUS, MAX_RT | TX_DS | RX_DR);
}

//if the nrf in rx/tx to standby mode it will reverse the action
void nrfRestorToRxTx()
// void nrfEnable()
{
  //This function will be called during operation
  //So after setting nrf_ce_high(), NRF will go to TX mode or RX mode
  //depending on the value of PRIM_RX bit
  nrf_ce_high();
}

void nrfRxTxToStandy1()
{
  //This function will be called during operation
  //So after setting nrf_ce_low(), NRF will go to Standby-1 whether it
  // is tx or rx mode
  nrf_ce_low();
}


/***********************************************************************************
 *                                         NRF Extended Feature
 * *********************************************************************************/

  //Enable of disable debug log in nrf 
  void nrfSetDebug(bool debugFlag)
  {
    _nrfDebug = debugFlag;
  }

  //Print nrf basic registers 
 void nrfPrintRegisters()
 {
    //serial_print("\r\n");
    //serial_print("\r\n All Registers\r\n");
    uint8_t i;
    uint8_t reg_array[11] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x17,0x1C,0x1D};
    for (i = 0; i < 11; i++)
    {
      SerialPrintU8(reg_array[i]);
      SerialPrint(" : ");
      SerialPrintlnU8(read_register(reg_array[i]));
      //serial_print_uint8(reg_array[i]); //serial_print(" : "); nrf_debug_register(reg_array[i]); //serial_print("\r\n");
    }
 }

