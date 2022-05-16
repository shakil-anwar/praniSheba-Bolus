
#include "flashMem.h"


void flashBegin(uint32_t spiSpeed)
{
  spi_begin(spiSpeed);
  //  _writeStatusReg(1, 0x00, NON_VOLATILE);
  //  _writeStatusReg(2, 0x00, NON_VOLATILE);
  csHigh();
}

void flashRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
  uint8_t *ptr = buf;
  _busyWait();
  csLow();
  spi_transfer(FLASH_READ_DATA);
  _spiSendAddr(addr);
  uint16_t i;
  for ( i = 0; i < len; i++)
  {
    ptr[i] = spi_transfer(0);
  }
  csHigh();
}

void flashWrite(uint32_t addr, uint8_t *buf, uint16_t len)
{
  uint8_t *ptr = buf;
  uint16_t valN;
//  uint16_t pageRem = 256 - (addr % 256);
  uint16_t pageRem = 256 - (uint16_t)(addr & 255UL);//uint16_t mod = divident & (divisor - 1);
//  SerialPrintF(P("Page Rem "));SerialPrintlnU16(pageRem);
  uint16_t offset = 0;
//  _writeEnable(NON_VOLATILE);
  _writeEnablefast();
  while (len > 0)
  {
    valN = (len <= pageRem) ? len : pageRem;
    _busyWait();
    csLow();
    spi_transfer(FLASH_PAGE_PROGRAM);
    _spiSendAddr(addr);
    uint16_t i;
    for (i = 0; i < valN; i++)
    {
      spi_transfer(ptr[i + offset]);
    }
    csHigh();
    addr += valN;
    offset += valN;
    len -= valN;
    pageRem = 256;
  }
}

void flashPrintBytes(uint8_t *buf, uint8_t len)
{
  uint8_t i;
  for (i = 0; i < len; i++)
  {
    SerialPrintU8(buf[i]);
    SerialPrint(" ");
  }
  SerialPrintln("");
}

void printPageBytes(uint8_t *pageBuf)
{
  uint16_t i;
  for ( i = 0; i < 256; i += 16)
  {
    flashPrintBytes(&pageBuf[i],16);
  }
}

void flashDumpPage(uint32_t addr, uint8_t *buf)
{
  addr = addr>>8;
  SerialPrintF(P("Dumping Page :"));SerialPrintlnU32(addr);
  flashRead(addr<<8, buf, 256);
  printPageBytes(buf);
}

void flashEraseChip()
{
//  if (_writeEnable(NON_VOLATILE))
//  {
    SerialPrintlnF(P("Erasing Chip.."));
    _writeEnablefast();
    csLow();
    spi_transfer(FLASH_CHIP_ERASE);
    csHigh();
    _writeDisable();
    _busyWait();
    SerialPrintlnF(P("Done"));
//  }
}

void flashEraseSector(uint32_t addr)
{
  _busyWait();
//  if (_writeEnable(NON_VOLATILE))
//  {
    _writeEnablefast();
    csLow();
    spi_transfer(FLASH_SECTOR_ERASE);
    _spiSendAddr(addr);
    csHigh();
    uint8_t status;
    //SerialPrintlnF(P("Write bit: "));
    do
    {
      status = _getStatus(WRITING_BIT);
      //      SerialPrintlnU8(status);
    } while (status);
//  }
}

void flashPowerDown()
{
  csLow();
  spi_transfer(FLASH_POWER_DOWN);
  csHigh();
}
void flashReleasePowerDown()
{
  csLow();
  spi_transfer(RELEASE_POWER_DOWN);
  csHigh();
  // __delay_cycles(3);//maximum 3 uS
  delayMicroseconds(3);
}
