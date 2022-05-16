#ifndef _NRF24_QUERY_CLIENT_H_
#define _NRF24_QUERY_CLIENT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include"nRF24_Qry_Common.h"
typedef void (*nrfMemFun_t)(uint32_t addr, uint8_t *data, uint16_t len);

    /*****************query Sensor Node Functions*******************/


bool nrfQueryBeginClient(volatile struct qryObj_t *qryObj);

uint8_t *nrfQuery(struct query_t *qry, void *bufPtr, uint8_t len);

struct pong_t *nrfping(struct query_t *qry, struct pong_t *ping);
uint32_t nrfPing();
uint32_t nrfPingSlot(uint16_t deviceId, uint8_t slotId,struct pong_t *pong);

bool nrfTxConfigHandler(uint16_t DeviceId,struct nrfNodeConfig_t *conf, uint32_t romAddr, nrfMemFun_t read, nrfMemFun_t save);
void nrfTxConfigReset(struct nrfNodeConfig_t *conf, uint16_t romAddr, nrfMemFun_t save);

void nrfTxSetModeClient(enum addrMode_t addrMode,struct nrfNodeConfig_t *conPt);
    



extern volatile struct qryObj_t nrfQryObj;




#ifdef __cplusplus
}
#endif

#endif
