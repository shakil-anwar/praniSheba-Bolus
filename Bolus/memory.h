#ifndef _MEMORY_H_
#define _MEMORY_H_

#include "./lib/mspDriver/mspDriver.h"
#include "./lib/mspIoT/mspIoT.h"
#include "Schema.h"




void flashPinSetup();

void memoryBegin();

void memq_test_read();
void memq_test_write();

void memReader(uint32_t addr, uint8_t *buf, uint16_t len);
void memWriter(uint32_t addr, uint8_t *buf, uint16_t len);
void memEraser(uint32_t addr, uint16_t len);
void memPtrReader(struct memqPtr_t *ptr);
void memPtrWriter(struct memqPtr_t *ptr);
void printBuffer(uint8_t *buf, uint8_t len);
uint8_t getSerialCmd();
void generatePld(payload_t *pld);
void printPld(payload_t *pld);
//uint8_t getSerialCmd();
void disableOthersOnbus();
void enableOthersOnbus();



void framRead(uint32_t addr, uint8_t *buf, uint16_t len);
void framUpdate(uint32_t addr, uint8_t *buf, uint16_t len);


void memoryReset();
void memoryFactoryReset();

extern struct memq_t memq;

#endif
