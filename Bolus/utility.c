/*
 * mspIoT.c
 *
 *  Created on: Mar 21, 2021
 *      Author: sshuv
 */

#include "utility.h"
#include "Param.h"

void bolusPowerDown()
{
    nrfPowerDown();
//    flashPowerDown();
}

void bolsuPowerUp()
{
//    flashReleasePowerDown();
    nrfStandby1();
}

uint32_t calcNextSlotUnix(uint32_t uSec, nrfNodeConfig_t *conf)
{
  uint16_t slotSec = (conf -> perNodeInterval) * (conf -> slotId);
  uint16_t curMoment = uSec % conf->momentDuration;

  uint32_t nexSlotSec;
  if (curMoment < slotSec)
  {
    nexSlotSec = uSec + (slotSec - curMoment);
  }
  else
  {
    nexSlotSec = uSec + (conf->momentDuration - curMoment) + slotSec;
  }
  SerialPrint("curMoment :"); SerialPrintlnU16(curMoment);
  SerialPrint("======>>>>>next slot unix :"); SerialPrintlnU32(nexSlotSec);
  return nexSlotSec;
}


void saveUnixTime()
{
    uint32_t unixTimeBekMem = *(uint32_t *)BKMEM_BASE;
    if(unixTimeBekMem > 1621838264){
        if(unixTimeBekMem < 2032065464)
        {
            FRAMWrite((uint8_t *)FRAM_TIME_SAVE_ADDRESS, (uint8_t *)BKMEM_BASE, 4);
        }
    }
}
uint32_t readUnixTime()
{
    uint32_t unixTimeBekMemRead = 0;
    FRAMRead((uint8_t *)FRAM_TIME_SAVE_ADDRESS, (uint8_t *)&unixTimeBekMemRead, 4);
    if(unixTimeBekMemRead > 1621838264){
        if(unixTimeBekMemRead < 2032065464)
        {
            return unixTimeBekMemRead;
        }
    }
    return NULL;
}



void FRAMWrite (uint8_t *framWritePtr, uint8_t *buf, uint16_t len)
{
    unsigned int i=0;
    uint8_t  *writePtr = framWritePtr;
   SerialPrint("fr>w>");

    SYSCFG0 = FRWPPW | PFWP;
    for (i = 0; i < len; i++)
    {
//        SerialPrintU8("*buf");
        *writePtr++ = *buf++;
    }
    SYSCFG0 = FRWPPW | PFWP | DFWP;
//    SerialPrintln("");
}

void FRAMRead (uint8_t *framReadPtr, uint8_t *buf, uint16_t len)
{
    unsigned int i=0;
    uint8_t  *writePtr = framReadPtr;
   SerialPrint("fr>r>");

//    SYSCFG0 = FRWPPW | PFWP;
    for (i = 0; i < len; i++)
    {
//        SerialPrintU8("*buf");
        *buf++ = *writePtr++;
    }
//    SerialPrintln("");
//    SYSCFG0 = FRWPPW | PFWP | DFWP;
}
