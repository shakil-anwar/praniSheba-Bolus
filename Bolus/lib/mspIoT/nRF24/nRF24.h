/*
    nrf24.h
    Created on: Aug 31, 2020
    Author: Shuvangkar Chandra Das & Shakil Anwar
*/
#ifndef _NRF24_H_
#define _NRF24_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "nRF24_Driver.h"
#include "nRF24_Registers.h"


enum nrf_irq_state_t
{
  NRF_WAIT,
  NRF_SUCCESS,
  NRF_RECEIVED,
  NRF_FAIL
};


enum nrf_mode_t
{
  UNDEFINED,
  POWER_DOWN,
  STANDBY1,
  STANDBY2,
  RXMODE,
  TXMODE
};

typedef void (*isrPtr_t)(void);


void nrfSetPin(volatile gpio_pin *cePort,volatile gpio_pin cePin,
              volatile gpio_pin *csnPort,volatile gpio_pin csnPin);
void nrfBegin(air_speed_t speed, power_t power, uint32_t spiSpeed);
void nrfSetTimers(uint32_t (*ms)(), uint32_t (*second)());
void nrfSetIrqs(isrPtr_t txIsr, isrPtr_t rxIsr, isrPtr_t maxRtIsr);
void nrfIrq();


void nrfSetPipe(uint8_t pipe, uint8_t *addr, bool ackFlag);
void nrfRxStart();


void nrfTxBegin(uint8_t *addr, bool ackFlag);
void nrfTxStart();
void nrtTxStartTransmission();


bool nrfIsRunning();

bool nrfIsRxEmpty();
bool nrfRxFifiFull();
uint8_t nrfReadStatus();
bool nrfIntStatus(uint8_t bitPos);
uint8_t pipeAvailable();
uint8_t pipeAvailFast();
bool nrfTxIsFifoEmpty();
enum nrf_mode_t nrfWhichMode();
void nrfPrintMode(enum nrf_mode_t mode);


void nrfWrite(const uint8_t *data, uint8_t len);
void nrfSend(const uint8_t *data, uint8_t len);
bool nrfAck();
bool nrfAckSend(const uint8_t *data, uint8_t len);
void nrfToTxAndSend(uint8_t *addr,const uint8_t *data, uint8_t len);



enum nrf_irq_state_t waitAck();

uint8_t *nrfRead(uint8_t *buffer, uint8_t len);
uint8_t nrfPayloadLen();


void nrfPrintRegisters();
void nrfSetDebug(bool debugFlag);

void nrfRxTxToStandy1();
void nrfRestorToRxTx();

void nrfStandby1();
void nrfPowerDown();

// #define nrfPowerDown()  clear_reg_bit(RF24_CONFIG, PWR_UP)
// #define nrfStandby1()   ({\
//                         set_reg_bit(RF24_CONFIG, PWR_UP);\
//                         delay(2);\
//                         })





#if defined(ARDUINO_ARCH_AVR)
  #define delay_us(us)     delayMicroseconds(us);
#elif defined(__MSP430__)
  #define delay_us(us)     __delay_cycles(us);
#else
  #error "Chip  not defined for delay_us"
#endif


extern volatile enum nrf_irq_state_t _nrfIrqState;
extern uint32_t (*_Millis)(void);
extern uint32_t (*_seconds)(void);
#ifdef __cplusplus
}
#endif

#endif
