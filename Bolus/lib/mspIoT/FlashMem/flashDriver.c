#include "flashDriver.h"
#include "FlashRegisters.h"

/*********************Global Variable *****************/
volatile gpio_t *_flashCsPort;
volatile gpio_t _flashCsPin;

void flashSetPin(volatile gpio_t *csPort, volatile gpio_t csPin)
{
  _flashCsPort = csPort;
  _flashCsPin = csPin;
#if defined(ARDUINO_ARCH_AVR)
  csPort--;
#elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
  csPort++;
#endif
  *csPort |= 1 << csPin;               // setting cs port ddr reg as output mode
  *_flashCsPort |= (1 << _flashCsPin); //chip disable
}


void csLow()
{
  *_flashCsPort &= ~(1 << _flashCsPin); //chip disable
}

void csHigh()
{
  *_flashCsPort |= (1 << _flashCsPin); //chip disable
}


uint8_t _readStatusReg(uint8_t regNo)
{
  csLow();
  switch (regNo)
  {
    case 1:
      spi_transfer(FLASH_READ_STATUS_1);
      break;
    case 2:
      spi_transfer(FLASH_READ_STATUS_2);
      break;
    case 3:
      spi_transfer(FLASH_READ_STATUS_3);
      break;
  }
  uint8_t reg = spi_transfer(0);
  csHigh();
  return reg;
}

void _writeStatusReg(uint8_t reg, uint8_t value, uint8_t memType)
{
  _busyWait();
  _writeEnable(memType);
  csLow();
  switch (reg)
  {
    case 1:
      spi_transfer(FLASH_WRITE_STATUS_1);
      break;
    case 2:
      spi_transfer(FLASH_WRITE_STATUS_2);
      break;
    case 3:
      spi_transfer(FLASH_WRITE_STATUS_3);
      break;
  }
  spi_transfer(value);
  csHigh();
  _writeDisable();
}

bool _writeEnable(uint8_t memType)
{
  /*
    volatile status register will affetct WEL register
    non-volatile write enable change WEL register.
  */
  csLow();
  switch (memType)
  {
    case NON_VOLATILE:
      spi_transfer(FLASH_WRITE_ENABLE);
      break;
    case VOLATILE:
      spi_transfer(FLASH_WR_ENA_VOLATILE);
      break;
  }
  csHigh();

  if (memType == NON_VOLATILE)
  {
    return _getStatus(WRITING_BIT);
  }
  return 0;
}


uint8_t _readStatusReg1()
{
  csLow();
  spi_transfer(FLASH_READ_STATUS_1);
  uint8_t reg = spi_transfer(0);
  csHigh();
  return reg;
}

bool _getStatus(uint8_t bit)
{
  //  uint8_t reg1 = _readStatusReg(1);
  uint8_t reg1 = _readStatusReg1();
  return ((reg1 & (1 << bit)) >> bit);
}

void _writeEnablefast()
{
  csLow();
  spi_transfer(FLASH_WRITE_ENABLE);
  csHigh();
}

void _writeDisable()
{
  csLow();
  spi_transfer(FLASH_WRITE_DISABLE);
  csHigh();
}

void _busyWait()
{
  bool busy = true;
  //    SerialPrintlnF(P("busy bit:"));
  do
  {
    busy = _getStatus(BUSY_BIT);
    //        SerialPrintlnU8(busy);
  } while (busy);
}



void _spiSendAddr(uint32_t addr)
{
  uint8_t *ptr = (uint8_t *)&addr;
  spi_transfer(ptr[2]);
  spi_transfer(ptr[1]);
  spi_transfer(ptr[0]);
}

uint16_t mod16(uint16_t divident, int8_t divisorPower)
{
  uint16_t  divisor = 1 << divisorPower; //so d will be 1, 2, 4, 8
  uint16_t mod = divident & (divisor - 1);  //divident%divisor = reminder
  return mod;
}
