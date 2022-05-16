#ifndef MSPFLASHMEMORY_MSPFLASH_H_
#define MSPFLASHMEMORY_MSPFLASH_H_

#ifdef __cplusplus 
extern "C" {
#endif

#include "flashDriver.h"
/******************NEW API****************************/
void flashBegin(uint32_t spiSpeed);

void flashRead(uint32_t addr, uint8_t *buf, uint16_t len);
void flashWrite(uint32_t addr, uint8_t *buf, uint16_t len);

void flashPrintBytes(uint8_t *buf, uint8_t len);
void flashDumpPage(uint32_t addr, uint8_t *buf); //print the whole page of addr

void flashEraseChip();
void flashEraseSector(uint32_t addr);

void flashPowerDown();
void flashReleasePowerDown();

#ifdef __cplusplus 
}
#endif
#endif /* MSPFLASHMEMORY_MSPFLASH_H_ */
