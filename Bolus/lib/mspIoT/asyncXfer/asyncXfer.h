#ifndef _ASYNC_XFER_H_
#define _ASYNC_XFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t *(*read_t)(void);
// typedef char *(*toJson_t)(uint8_t*);
typedef void (*send_t)(uint8_t*);
typedef int (*ackWait_t)(void);
typedef uint32_t (*millis_t)(void);

void xferBegin(read_t read, send_t send, ackWait_t ackWait, millis_t ms);
void xferConfig(uint8_t mxTry,bool debug);


void xferReady();
bool xferSendLoop();
bool xferSendLoopV2();
bool xferSendLoopV3();

#ifdef __cplusplus
}
#endif
#endif /*End _ASYNC_XFER_H_*/
