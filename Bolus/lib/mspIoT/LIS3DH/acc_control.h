#ifndef ACC_CONTROL_H_
#define ACC_CONTROL_H_

#include "mspAcc.h"
#include "Schema.h"
//#include "mspPayload.h"

#define ACC_MAX_POINT 32
// void accSetBuffer(payload_t **pldBuf);

// payload_t *getAccdata(payload_t *ptr);

//void *readAcc(payload_t *ptr);
void accIsr();


#endif
