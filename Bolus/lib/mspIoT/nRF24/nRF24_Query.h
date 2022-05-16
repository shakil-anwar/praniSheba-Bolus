// #ifndef _NRF24_QUERY_H_
// #define _NRF24_QUERY_H_

// #ifdef __cplusplus
// extern "C"
// {
// #endif

// #include "nRF24.h"


//     typedef void (*nrfMemFun_t)(uint32_t addr, uint8_t *data, uint16_t len);

//      typedef struct pipeAddr_t
//     {
//         uint8_t pipe0Addr[5];
//         uint8_t pipe1Addr[5];
//         uint8_t pipe2Addr;
//         uint8_t pipe3Addr;
//         uint8_t pipe4Addr;
//         uint8_t pipe5Addr;
//     }pipeAddr_t;

//     enum deviceType_t
//     {
//         BASE_STATION,
//         SENSOR_NODE
//     };

//     typedef struct query_t
//     {
//         uint16_t deviceId;
//         uint8_t slotId;
//         uint8_t type;
//         uint8_t opcode;
//         uint8_t checksum;
//     } query_t;
    

//     typedef struct pong_t
//     {
//         uint32_t second;
//         uint32_t ms;

//         uint8_t type;
//         uint8_t opcode;

//         bool isConfigChanged;  
//         bool isBsFree;
//         bool isMySlot;
      
//         uint8_t checksum;
//     } pong_t;

//   /*
//         ==========================================
//         isBsFree    |   isMyslot    |   okToSend
//         ------------------------------------------
//             0       |        0      |         0
//             0       |        1      |         0           
//             1       |        0      |         1
//             1       |        1      |         1
//         ==========================================
//  */

//     typedef struct nrfNodeConfig_t
//     {
//         // uint16_t deviceId;
//         uint16_t momentDuration;
//         uint16_t perNodeInterval;
//         uint8_t slotId;
//         uint8_t node[5];
//         uint8_t type;
//         uint8_t opcode;
//         uint8_t dataPipeLsbByte; //for making the structure even byte
//         uint8_t checksum;
//     } nrfNodeConfig_t;

//     struct qryObj_t
//     {
//         //common param
//         uint8_t pipe;  //query pipe no
//         uint8_t *addr; //query common addr
//         //Base station param
//         uint8_t *bufPtr; //query buffer
//         uint8_t len;     //query buffer length
//         pipeAddr_t *pipeAddr;
//         uint8_t *(*resolver)(uint8_t *);
//     };

//     enum addrMode_t
//     {
//         COMMON_PING,
//         BS_PING,
//         BS_DATA
//     };

   

//     bool nrfQueryBegin(volatile struct qryObj_t *qryObj, enum deviceType_t devType);

//     /*****************query Sensor Node Functions*******************/
//     uint8_t *nrfQuery(query_t *qry, void *bufPtr, uint8_t len);
    
//     pong_t *nrfping(query_t *qry, pong_t *ping);
//     uint32_t nrfPing();
//     uint32_t nrfPingSlot(uint16_t deviceId, uint8_t slotId);

//     bool nrfTxConfigHandler(uint16_t DeviceId, nrfNodeConfig_t *conf, uint16_t romAddr, nrfMemFun_t read, nrfMemFun_t save);
//     void nrfTxConfigReset(nrfNodeConfig_t *conf, uint16_t romAddr, nrfMemFun_t save);
//     void nrfTxReady(nrfNodeConfig_t *conPtr);



    
    
//     void nrfTxSetMode(enum addrMode_t addrMode,nrfNodeConfig_t *conPt);
//     /*****************query Server Functions*******************/
//     void nrfQueryHandler(query_t *query);
//     void nrfQryServerEnd(query_t *qry);

//     void nrfServerConfigResolver(query_t *qry);


//     extern volatile struct qryObj_t nrfQryObj;

// #ifdef __cplusplus
// }
// #endif

// #endif
