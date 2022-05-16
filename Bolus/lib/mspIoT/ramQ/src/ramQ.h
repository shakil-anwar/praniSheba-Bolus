#ifndef _RAMQ_H_
#define _RAMQ_H_
//#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


void ramQSet(void *bufPtr, uint8_t packetSz, uint8_t totalPacket); //ramQset(&payload[0],sizeof(payload_t),TOTAL_PAYLOAD)

void *ramqGetHead();
void *ramqGetTail();
bool ramqIsLocked();
uint16_t ramqAvailable();

void *ramQHead();
void *ramQUpdateHead();// increment current head pointer and return head pointer.

void *ramqGetNextHead();
bool ramqUpdate();

void *ramQRead();
void *ramQReadTail();
void *ramQUpdateTail();


void ramqPrintLog();

extern volatile void *_ramQHead;
extern volatile void *_ramQTail;
extern volatile void *_ramQFlash;
extern volatile void *_ramQBase;

extern volatile int8_t ramQCounter;
extern volatile int8_t ramQTailCounter;
#ifdef __cplusplus
}
#endif

#endif
