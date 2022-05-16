// #include "nRF24_Query.h"

// #if defined(ARDUINO_ARCH_AVR)
//     #include <Arduino.h>
//     #if defined(PROD_BUILD)
//         #include "../Sensor_TDM/TDM.h"
//     #else
//         #include "TDM.h"
//     #endif
// #elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
//       #include "mspIoT.h"
// #else
//     #error "nRF24_DRIVER did not find chip architecture "
// #endif

// #define QUERY_TIMEOUT_MS 1000
// #define PING_TYPE 200
// #define PING_OPCODE 201
// #define PING2_OPCODE 203
// #define ADDR_OPCODE 221
// #define NRF_CONFIG_OPCODE 225


// uint8_t *nrfReadQuery(uint8_t *ptr, uint8_t len);
// void PongHandler(query_t *query);
// nrfNodeConfig_t *nrfTxGetConfig(uint16_t DeviceId, nrfNodeConfig_t *configPtr);

// void printPing( pong_t *pingPtr);
// void nrfPrintConfig(nrfNodeConfig_t *nrfConf);

// void setBsDataMode(nrfNodeConfig_t *conPt);

// /**********************Global Vars*************************/
// uint8_t _txAddr[5];
// volatile struct qryObj_t nrfQryObj;
// uint8_t _freeSlotId;
// enum deviceType_t deviceType;
// uint8_t *pipe0Addr;
// /**********************Common function***************************************/

// bool nrfQueryBegin(volatile struct qryObj_t *qryObj, enum deviceType_t devType)
// {
//     bool isOk;
//     pipe0Addr = qryObj -> pipe;
//     deviceType = devType;
    
//     isOk =  (qryObj -> pipe >= 0 && qryObj -> pipe < 6) && 
//             (qryObj -> addr != NULL) &&
//             (_seconds != NULL) &&
//             (_Millis != NULL);
//     if(devType == BASE_STATION)
//     {
//         isOk =  isOk && 
//             (qryObj -> bufPtr != NULL) && 
//             (qryObj -> len > 0 && qryObj -> len <= 32) &&
//             (qryObj -> pipeAddr != NULL) &&
//             (qryObj -> resolver !=  NULL);
//         nrfSetPipe(qryObj -> pipe, qryObj -> addr, true);
//     }

//     if(isOk == false)
//     {
//         SerialPrintF(P("NRF QRY Begin Failed"));
//     }
//     return isOk;
// }



// void nrfTxSetMode(enum addrMode_t addrMode,nrfNodeConfig_t *conPt)
// {
//     //  nrfTxBegin(conPtr -> node, true);
//     // nrfStandby1();
//     // nrfTxStart();
//     // if(_nrfDebug)
//     // {
//     //     SerialPrintlnF(P("TX Ready"));
//     // }     
//     switch (addrMode)
//     {
//     case COMMON_PING:
//         if(deviceType == BASE_STATION)
//         {
//             nrfQryObj.addr = nrfQryObj.pipeAddr->pipe0Addr;
//         }
//         else
//         {
//             nrfQryObj.addr = pipe0Addr;
//         }
        
//         break;
//     case BS_PING:
//         if(deviceType == BASE_STATION)
//         {
//             nrfQryObj.addr = nrfQryObj.pipeAddr->pipe1Addr;
//         }
//         else
//         {
//             nrfQryObj.addr = conPt->node;
//         }
//         break;
//     case BS_DATA:
//         //concatenate pipe 2 addr
//         setBsDataMode(conPt);
//         break;
//     default:
//         break;
//     }
// }


// void setBsDataMode(nrfNodeConfig_t *conPt)
// {
//     uint8_t dataPipeAddr[5];
//     strncpy((char *)dataPipeAddr,(char *)conPt -> node,5);
//     dataPipeAddr[4]= conPt -> dataPipeLsbByte;
//     nrfTxBegin(dataPipeAddr, true);
//     nrfStandby1();
//     nrfTxStart();
//     if(_nrfDebug)
//     {
//         SerialPrintlnF(P("TX Ready"));
//     } 
// }
// /*********************************Sensor node funstions Definitions**********************/


// uint8_t *nrfReadQuery(uint8_t *ptr, uint8_t len)
// {
//     switch (_nrfIrqState)
//     {
//     case NRF_SUCCESS:
//         // nrfTxBegin(_mainTxAddr,true);//After query send change tx address to previous tx addr.

//         nrfSetPipe(nrfQryObj.pipe, nrfQryObj.addr, true); //Change to rx mode
//         nrfRxStart();
//         #if defined(ARDUINO_ARCH_AVR)
//         // if (_nrfDebug)
//         // {
//         //     SerialPrintlnF(P("NRF Query Accepted"));
//         // }
//         #endif
//         _nrfIrqState = NRF_WAIT;
//         break;
//     case NRF_WAIT:
//         // //serial_print("\r\n Q_WAIT");
//         break;
//     case NRF_RECEIVED:
//         #if defined(ARDUINO_ARCH_AVR)
//         // if (_nrfDebug)
//         // {
//         //     SerialPrintlnF(P("NRF Query Received"));
//         // }
//         #endif
//         // SerialPrintF(P("Len : "));SerialPrintlnU8(nrfPayloadLen());
//         nrfRead(ptr, len);
//         return ptr;
//         break;
//     }
//     return NULL;
// }

// uint8_t *nrfQuery(query_t *qry,void *bufPtr, uint8_t len)
// {
//     if(_nrfDebug){SerialPrintlnF(P("Querying.."));}
    
//     nrfToTxAndSend(nrfQryObj.addr,(uint8_t *)qry, sizeof(query_t));

//     uint32_t currentMillis = _Millis();
//     uint32_t prevMillis = currentMillis;
//     uint8_t *queryPtr = NULL;
//     do
//     {
//         queryPtr = nrfReadQuery((uint8_t*)bufPtr,len);
//         if (queryPtr != NULL)
//         {
//             nrfPrintBuffer(queryPtr, len);
//             len--;//pointing last byte
//             uint8_t calcCheckSum = checksum(queryPtr,len);
//             if(calcCheckSum == queryPtr[len])
//             {
//                 if(_nrfDebug){SerialPrintlnF(P("<Qry Ok>"));} 
//             }
//             else
//             {
//                 queryPtr = NULL;
//                 if(_nrfDebug)
//                 {
//                     SerialPrintlnF(P("<Qry Mismatch>"));
//                     SerialPrintF(P("Expected : ")); SerialPrintU8(queryPtr[len]);
//                     SerialPrintF(P(" | Received : ")); SerialPrintlnU8(calcCheckSum);
//                 }
//             }
//             break;
//         }      
//         currentMillis = _Millis();
//     } while ((currentMillis - prevMillis) < QUERY_TIMEOUT_MS);

//     if(queryPtr == NULL)
//     {
//         if(_nrfDebug){SerialPrintlnF(P("<Qry Failed>"));}
//     }
//     return queryPtr;
// }


// pong_t *nrfping(query_t *qry,pong_t *pong)
// {
//   nrfStandby1();
//   nrfTxStart();
  
//   pong_t *pongPtr = (pong_t *)nrfQuery(qry,(void *)pong,sizeof(pong_t));
//   if (pongPtr != NULL)
//   {
//       printPing(pongPtr);
//   }
//   return pongPtr;
// }

// uint32_t nrfPing()
// {
//     query_t query;
//     pong_t pong;

//     query.type = PING_TYPE;
//     query.opcode = PING_OPCODE;
//     pong_t *ponPtr = nrfping(&query,&pong);
//     if(ponPtr != NULL)
//     {
//         return ponPtr -> second;
//     }
//     return 0;
// }

// uint32_t nrfPingSlot(uint16_t deviceId, uint8_t slotId)
// {
//     query_t query;
//     pong_t pong;

//     query.type = PING_TYPE;
//     query.opcode = PING2_OPCODE;
//     query.deviceId = deviceId;
//     query.slotId = slotId;

//     pong_t *ponPtr = nrfping(&query,&pong);
//     if(ponPtr != NULL)
//     {
//          /*
//         ==========================================
//         isBsFree    |   isMyslot    |   okToSend
//         ------------------------------------------
//             0       |        0      |         0
//             0       |        1      |         0           
//             1       |        0      |         1
//             1       |        1      |         1
//         ==========================================
//         */
// #if defined(ARDUINO_ARCH_AVR)
//        if(ponPtr ->isBsFree && ponPtr ->isMySlot)
//        {
//            return ponPtr -> second;
//        }
//     }
//     return NULL;
// #elif defined(__MSP430G2553__) || defined(__MSP430FR2433__)
//        if(ponPtr ->isBsFree && ponPtr ->isMySlot)
//        {
//            return 0;
//        }else
//        {
//            return ponPtr -> second;
//        }
//         // return (ponPtr ->isBsFree && ponPtr ->isMySlot);
//     }
//     return 1;
// #endif


// }

// void printPing( pong_t *pingPtr)
// { 
//     SerialPrintlnF(P("<------Ping------>"));
//     // SerialPrintF(P("Devide Id : "));SerialPrintlnU32(pingPtr -> deviceId);
//     SerialPrintF(P("Sec : "));SerialPrintlnU32(pingPtr -> second);
//     SerialPrintF(P("ms : "));SerialPrintlnU32(pingPtr -> ms);

//     SerialPrintF(P("type : "));SerialPrintlnU8(pingPtr -> type);  
//     SerialPrintF(P("opcode : "));SerialPrintlnU8(pingPtr -> opcode);


//     SerialPrintF(P("isBsFree : "));SerialPrintlnU32(pingPtr -> isBsFree);
//     SerialPrintF(P("isMySlot : "));SerialPrintlnU32(pingPtr -> isMySlot);
//     SerialPrintF(P("isConfigChanged: "));SerialPrintlnU32(pingPtr -> isConfigChanged);

//     SerialPrintF(P("Checksum : "));SerialPrintlnU8(pingPtr -> checksum);  
      
// }

// void nrfTxReady(nrfNodeConfig_t *conPtr)
// {
//     nrfTxBegin(conPtr -> node, true);
//     nrfStandby1();
//     nrfTxStart();
//     if(_nrfDebug)
//     {
//         SerialPrintlnF(P("TX Ready"));
//     }     
// }

// void nrfPrintConfig(nrfNodeConfig_t *nrfConf)
// {
//     SerialPrintlnF(P("<------Config------>"));
//     // SerialPrintF(P("deviceId : "));SerialPrintlnU32(nrfConf -> deviceId);
//     SerialPrintF(P("momentDuration : "));SerialPrintlnU32(nrfConf -> momentDuration);
//     SerialPrintF(P("perNodeInterval : "));SerialPrintlnU8(nrfConf -> perNodeInterval);
//     SerialPrintF(P("slotId : "));SerialPrintlnU8(nrfConf -> slotId);

//     SerialPrintF(P("Addr : ")); nrfPrintBuffer(nrfConf -> node, sizeof(nrfConf -> node));
//     SerialPrintF(P("DataPipeLSB : "));SerialPrintlnU8(nrfConf -> dataPipeLsbByte);
//     SerialPrintF(P("type : "));SerialPrintlnU8(nrfConf -> type);
//     SerialPrintF(P("opcode : "));SerialPrintlnU8(nrfConf -> opcode);
//     SerialPrintF(P("Checksum : "));SerialPrintlnU8(nrfConf -> checksum); 
// }

// nrfNodeConfig_t *nrfTxGetConfig(uint16_t DeviceId, nrfNodeConfig_t *configPtr)
// {
// //   nrfNodeConfig_t addr;
//   nrfStandby1();
//   nrfTxStart();
//   query_t query;
//   query.type = PING_TYPE;
//   query.opcode = NRF_CONFIG_OPCODE;
//   query.deviceId = DeviceId;

//   configPtr = nrfQuery(&query,configPtr,sizeof(nrfNodeConfig_t));
//   if (configPtr != NULL)
//   {
//     nrfPrintConfig(configPtr);
//   }
//    return configPtr;
// }

// bool nrfTxConfigHandler(uint16_t DeviceId, nrfNodeConfig_t *conf,
//                         uint16_t romAddr,nrfMemFun_t read,nrfMemFun_t save)
// {
//     read((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
//     //match checksum to validate that data does erased in memory
//     uint8_t checksumCalc = checksum(conf, sizeof(nrfNodeConfig_t)-1);
//     bool isConfOk = (checksumCalc == conf->checksum) && 
//                     (conf -> type == PING_TYPE) &&
//                     (conf -> opcode == NRF_CONFIG_OPCODE);
//     if(isConfOk == false)                           
//     {
//         //Device has no config, or erased, get the new config
//         if(_nrfDebug){ SerialPrintlnF(P("Getting New Config"));}
//         conf = nrfTxGetConfig(DeviceId,conf);
//         if(conf !=NULL)
//         {
//             save((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
//             return true;
//         }
//         else
//         {
//             //config qry failed
//             return false;
//         }
//     }
//     else
//     {
//         if(_nrfDebug)
//         { 
//             SerialPrintlnF(P("Device has Config"));
//             nrfPrintConfig(conf);
//         }
//     }
//     return isConfOk;
// }

// void nrfTxConfigReset(nrfNodeConfig_t *conf, uint16_t romAddr,nrfMemFun_t save)
// {
//     memset(conf,0,sizeof(nrfNodeConfig_t));
//     save((uint32_t)romAddr,(uint8_t*)conf,sizeof(nrfNodeConfig_t));
// }


// /**********************NRF Server Side functions**************************/

// void nrfQueryHandler(query_t *query)
// {
//     // if (_nrfDebug)
//     // {
//     //     SerialPrintF(P("Qry type :")); SerialPrintU8(query.type);
//     //     SerialPrintF(P(" | Qry opcode :")); SerialPrintlnU8(query.opcode);
//     // }
//     // if(nrfQryObj.pipe > 0)
//     // {
//     //     nrfTxSetMode(BS_PING,)
//     // }
    
//     if(query -> type == PING_TYPE)
//     {
        
//      //Handle Internal query 
//      switch(query -> opcode)
//      {
//         case PING_OPCODE:
//         case PING2_OPCODE:
//             PongHandler(query);
//             break;
//         case NRF_CONFIG_OPCODE:
//             nrfServerConfigResolver(query);
//             break;
//      }
//     }
//     else
//     {
//         nrfQryObj.resolver(nrfQryObj.bufPtr);
//         nrfToTxAndSend(nrfQryObj.addr, nrfQryObj.bufPtr, nrfQryObj.len);
//     }
// }


// void nrfQryServerEnd(query_t *qry)
// {
//     //call this function in txIrq and mxRtIrq
//     nrfRxStart();
//     if(_nrfIrqState == NRF_SUCCESS)
//     {
//         SerialPrintlnF(P("<Qry Rcvd>"));
//         if(qry -> opcode == NRF_CONFIG_OPCODE)
//         {
//            bool ok = tdmConfirmSlot(_freeSlotId);
//         }
//     }
//     else
//     {
//         SerialPrintlnF(P("<Qry Failed>"));
//     }
// }


// void PongHandler(query_t *qry)
// {  
//     pong_t pong;
//     //common pong data
//     pong.type = PING_TYPE;
//     pong.opcode = qry->opcode;
//     pong.second = _seconds();
//     pong.ms = _Millis();
    
//     //specific pong data
//     if(qry->opcode == PING2_OPCODE)
//     {
//         struct node_t *currentNode = (struct node_t*)tdmGetCurrentNode();
//         if(currentNode != NULL)
//         {
//             SerialPrintF(P("Slot id : "));SerialPrintlnU8(currentNode -> slotNo);
//             pong.isBsFree = true;
//             pong.isMySlot = currentNode -> slotNo == qry ->slotId;
//             pong.isConfigChanged = false;
//         }
//         else
//         {
//             pong.isBsFree = false;
//             pong.isMySlot = false;
//             pong.isConfigChanged = false;
//             SerialPrintlnF(P("Slot not synced"));
//         }
        
//     }
//     pong.checksum = checksum(&pong, sizeof(pong_t)-1);
//     // pong.checksum = 10;
//     nrfToTxAndSend(nrfQryObj.addr, (const uint8_t *)&pong, sizeof(pong_t));

// }

// void nrfServerConfigResolver(query_t *qry)
// { 
//     SerialPrintlnF(P("Resolving Config"));
//     nrfNodeConfig_t config;
//     // config.deviceId = qry -> deviceId;
//     config.type = qry -> type;
//     config.opcode = qry -> opcode;
    
//     // uint8_t *addrPtr =  nrfQryObj.generateAddr(); //server generates adddress 
//     memcpy(config.node, nrfQryObj.pipeAddr->pipe1Addr,sizeof(config.node));
//     config.dataPipeLsbByte = nrfQryObj.pipeAddr->pipe2Addr;

//     struct tdmMeta_t *tdmMeta = tdmGetMetaData();
//     config.momentDuration = tdmMeta -> momentDuration;
//     config.perNodeInterval = tdmMeta -> perNodeInterval;
//     config.slotId =  tdmGetFreeSlot(qry -> deviceId);
//     _freeSlotId = config.slotId;

//     config.checksum = checksum(&config, sizeof(nrfNodeConfig_t)-1);

//     nrfToTxAndSend(nrfQryObj.addr, (const uint8_t *)&config, sizeof(nrfNodeConfig_t));
//     nrfPrintBuffer((uint8_t*)&config, sizeof(nrfNodeConfig_t));
// }

















// /***************************Old api**********************************/

// // void nrfGetAddr(nrfNodeConfig_t *addrPtr)
// // {
// // //   nrfNodeConfig_t addr;
// //   nrfStandby1();
// //   nrfTxStart();
// //   query_t query;
// //   query.type = PING_TYPE;
// //   query.opcode = ADDR_OPCODE;

// //   addrPtr = nrfQuery(&query,addrPtr,sizeof(nrfNodeConfig_t));
// //   if (addrPtr != NULL)
// //   {
// //     // SerialPrintF(P("Addr : "));
// //     // nrfPrintBuffer(addrPtr -> node, sizeof(addrPtr -> node));

// //     // SerialPrintF(P("type : "));SerialPrintlnU8(addrPtr -> type);
// //     // SerialPrintF(P("opcode : "));SerialPrintlnU8(addrPtr -> opcode);
// //     // SerialPrintF(P("Checksum : "));SerialPrintlnU8(addrPtr -> checksum);
// //     if(checksum(addrPtr, sizeof(nrfNodeConfig_t)-1) != addrPtr -> checksum)
// //     {
// //         if(_nrfDebug)
// //         {
// //             SerialPrintlnF(P("Addr Qry checksum mismatch"));
// //         }
// //     }
// //   }
// //   else
// //   {
// //     if(_nrfDebug)
// //     {
// //         SerialPrintF(P("Addr Qry Failed"));
// //     }
// //   } 
// // }


// // bool nrfTxHasAddr(nrfNodeConfig_t *addrPtr, addrFun_t read)
// // {
// //     read(addrPtr);
// //     if(addrPtr -> type == PING_TYPE && addrPtr -> opcode == ADDR_OPCODE)
// //     {
// //         // SerialPrintlnF(P("Device has address"));
// //         return true;
// //     }
// //     return false;
// // }

// // void nrfTxAddrHandler(addrFun_t read, addrFun_t save)
// // {
// //     //read addr struct from memory
// //     nrfNodeConfig_t addr;
// //     bool hasAddr = nrfTxHasAddr(&addr,read);
// //     if(hasAddr == false)
// //     {
// //         if(_nrfDebug){ SerialPrintlnF(P("Getting New Addr"));}
// //         nrfGetAddr(&addr);
// //         save(&addr);
// //     }
// //     //validate address using checksum 
// //     memcpy(_txAddr,addr.node,sizeof(addr.node));
// //     if(_nrfDebug)
// //     {
// //         SerialPrintF(P("TX Addr : ")); 
// //         nrfPrintBuffer(_txAddr,sizeof(_txAddr));
// //     }
// // }

// // void nrfTxAddrReset(addrFun_t save)
// // {
// //     if(_nrfDebug)
// //     {
// //         SerialPrintlnF(P("Resetting TX Addr"));
// //     }
// //     nrfNodeConfig_t addr;
// //     save(&addr);
// // }


// // void nrfServerAddrResolve(query_t *qry)
// // {
// //     nrfNodeConfig_t addr;

// //     // addr.node[0] = 1;
// //     // addr.node[1] = 2;
// //     // addr.node[2] = 3;
// //     // addr.node[3] = 4;
// //     // addr.node[4] = 5;
// //     uint8_t *addrPtr =  nrfQryObj.generateAddr(); //server generates adddress 
// //     memcpy(addr.node, addrPtr,sizeof(addr.node));

// //     addr.type = qry -> type;
// //     addr.opcode = qry-> opcode;
// //     addr.checksum = checksum(&addr, sizeof(nrfNodeConfig_t)-1);
    
// //     // SerialPrintF(P("type : "));SerialPrintlnU8(addr.type);
// //     // SerialPrintF(P("opcode : "));SerialPrintlnU8(addr.opcode);
// //     // SerialPrintF(P("Checksum : "));SerialPrintlnU8(addr.checksum);

// //     nrfToTxAndSend(nrfQryObj.addr, (const uint8_t *)&addr, sizeof(nrfNodeConfig_t));
// // }

// // void nrfTxAddrRestore(uint8_t lsByte)
// // {
// //     uint8_t addr[5];
// //     addr[0] = lsByte;

// //     memcpy(&addr[1], &nrfQryObj.addr[1], 4);
// //     nrfTxBegin(addr, true);
// // }


// // void pongHandler(query_t qry)
// // {
// //     pong_t ping;
// //     //attach second with pong 
// //     ping.second = _seconds();
// //     ping.ms = _Millis();
 
    
// //     ping.type = qry.type;
// //     ping.opcode = qry.opcode;
// //     // ping.padding = 0;
// //     ping.checksum = checksum(&ping, sizeof(pong_t)-1);
    
// //     // SerialPrintF(P("second : "));SerialPrintlnU32(ping.second);
// //     // SerialPrintF(P("ms : "));SerialPrintlnU32(ping.ms);
// //     // SerialPrintF(P("type : "));SerialPrintlnU8(ping.type);
// //     // SerialPrintF(P("opcode : "));SerialPrintlnU8(ping.opcode);
// //     // SerialPrintF(P("Checksum : "));SerialPrintlnU8(ping.checksum);

// //     nrfToTxAndSend(nrfQryObj.addr, (const uint8_t *)&ping, sizeof(pong_t));
// // }


// // uint32_t nrfPing()
// // {
// //   pong_t ping;
// //   nrfStandby1();
// //   nrfTxStart();
  
// //    query_t query;
// //    query.type = PING_TYPE;
// //    query.opcode = PING_OPCODE;

// //   pong_t *pingPtr = nrfQuery(&query,&ping,sizeof(pong_t));
// //   if (pingPtr != NULL)
// //   {
// //     SerialPrintF(P("Ping Sec : "));SerialPrintlnU32(pingPtr -> second);
// //     // SerialPrintF(P("ms : "));SerialPrintlnU32(pingPtr -> ms);
// //     // SerialPrintF(P("type : "));SerialPrintlnU8(pingPtr -> type);
// //     // SerialPrintF(P("opcode : "));SerialPrintlnU8(pingPtr -> opcode);
// //     // SerialPrintF(P("Checksum : "));SerialPrintlnU8(pingPtr -> checksum);
// //     if(checksum(&ping, sizeof(pong_t)-1) != pingPtr -> checksum)
// //     {
// //         if(_nrfDebug)
// //         {
// //           SerialPrintlnF(P("Checksum mismatch"));
// //         }
// //     }   
// //     return pingPtr -> second;
// //   }
// //   else
// //   {
// //     if(_nrfDebug)
// //     {
// //         SerialPrintlnF(P("Ping Failed"));
// //     }
// //     return 0;
// //   } 
// // }
