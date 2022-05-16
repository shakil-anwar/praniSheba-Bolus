#include "nRF24_Qry_Common.h"

volatile struct qryObj_t nrfQryObj;

void nrfTxReady(nrfNodeConfig_t *conPtr)
{
    nrfTxBegin(conPtr -> node, true);
    nrfStandby1();
    nrfTxStart();
    if(_nrfDebug)
    {
        SerialPrintlnF(P("TX Ready"));
    }     
}


void printPing( pong_t *pingPtr)
{ 
    SerialPrintlnF(P("<------Ping------>"));
    // SerialPrintF(P("Devide Id : "));SerialPrintlnU32(pingPtr -> deviceId);
    SerialPrintF(P("Sec : "));SerialPrintlnU32(pingPtr -> second);
    SerialPrintF(P("ms : "));SerialPrintlnU32(pingPtr -> ms);

    SerialPrintF(P("type : "));SerialPrintlnU8(pingPtr -> type);  
    SerialPrintF(P("opcode : "));SerialPrintlnU8(pingPtr -> opcode);


    SerialPrintF(P("isBsFree : "));SerialPrintlnU32(pingPtr -> isBsFree);
    SerialPrintF(P("isMySlot : "));SerialPrintlnU32(pingPtr -> isMySlot);
    SerialPrintF(P("isConfigChanged: "));SerialPrintlnU32(pingPtr -> isConfigChanged);

    SerialPrintF(P("Checksum : "));SerialPrintlnU8(pingPtr -> checksum);  
      
}


void nrfPrintConfig(nrfNodeConfig_t *nrfConf)
{
    SerialPrintlnF(P("<------Config------>"));
    // SerialPrintF(P("deviceId : "));SerialPrintlnU32(nrfConf -> deviceId);
    SerialPrintF(P("momentDuration : "));SerialPrintlnU32(nrfConf -> momentDuration);
    SerialPrintF(P("perNodeInterval : "));SerialPrintlnU8(nrfConf -> perNodeInterval);
    SerialPrintF(P("slotId : "));SerialPrintlnU8(nrfConf -> slotId);

    SerialPrintF(P("Addr : ")); nrfPrintBuffer(nrfConf -> node, sizeof(nrfConf -> node));
    SerialPrintF(P("DataPipeLSB : "));SerialPrintlnU8(nrfConf -> dataPipeLsbByte);
    SerialPrintF(P("type : "));SerialPrintlnU8(nrfConf -> type);
    SerialPrintF(P("opcode : "));SerialPrintlnU8(nrfConf -> opcode);
    SerialPrintF(P("Checksum : "));SerialPrintlnU8(nrfConf -> checksum); 
}
