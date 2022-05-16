 #include "memory.h"
#include "Pin.h"
#include "radio.h"
#include "utility.h"

//int data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
//data_t dataStruct[8]; //main data buffer

//data_t Data; //buffer for writing one sample
//data_t readBuf;//buffer for reading one sample

uint8_t *FRAM_write_ptr;


struct memqPtr_t ring;

payload_t pldWrite;
payload_t pldRead;

uint8_t pageBuf[256];
//struct memq_t *memq;
struct memq_t memq;


void memoryBegin()
{
//    flashSetPin(&P3OUT, 1);
    flashBegin(100000);
    flashPowerDown();
//    flashReleasePowerDown();
    // delay(100);
//    eraseChip();
    ring._head = 0;
    ring._tail = 0;

    // Data.val1 = 0;
    // Data.val2 = 1;
    FRAM_write_ptr = (uint8_t *)FRAM_TEST_START;
//    memq = memqNew(0, sizeof(payload_t), TOTAL_FLASH_BUFFER);
//    memq -> setMemory(memq, memReader, memWriter, memEraser, 4096);
//    memq -> setPointer(memq, memPtrReader, memPtrWriter,5);
//    memq -> attachBusSafety(memq, enableOthersOnbus,disableOthersOnbus);

    memqBegin(&memq,0, 32, TOTAL_FLASH_BUFFER);
    memqSetMem(&memq, memReader, memWriter, memEraser, 4096);
    memqSetMemPtr(&memq, memPtrReader, memPtrWriter, 5);

#if defined(RESET_MEMORY)
//    flashReleasePowerDown();
////    memq -> reset(memq);
//    memqReset(&memq);
//    flashPowerDown();
    memoryReset();
#endif

#if defined(FACTORY_RESET)
    memoryFactoryReset();
#endif
}

void memoryReset()
{
    flashReleasePowerDown();
    memqReset(&memq);
    flashPowerDown();
}

void memoryFactoryReset()
{
    memoryReset();
    nrfTxConfigReset(&nrfConfig, FRAM_NRF_DATA_SEND_ADDRESS, framUpdate);
}


void memq_test_write()
{
    if(memqIsLock(&memq) == true)
    {
        memq_test_read();
    }else
    {
        generatePld(&pldWrite);
        memqWrite(&memq, (uint8_t*)&pldWrite); //writing single data point
        printPld(&pldWrite);
        uint32_t curPage = (memq.ringPtr._head);
        flashDumpPage(curPage, pageBuf);
        SerialPrint("Counter : ");SerialPrintU16(memq._ptrEventCounter);
        SerialPrintln("");
    }
//    SerialPrint("\r\nFRAM WRITE STARTS: ");
//    SerialPrintU32(millis());
    memqSaveMemPtr(&memq);
//    SerialPrint("\r\nFRAM WRITE ENDS: ");
//    SerialPrintU32(millis());
}

//void memq_test_read()
//{
//        pldRead = memqRead(&memq, (uint8_t*)&pldRead);
//        printPld(&pldRead);
//        SerialPrint("Counter : ");SerialPrintU16(memq._ptrEventCounter);
//        SerialPrintln("");
//}



void memReader(uint32_t addr, uint8_t *buf, uint16_t len)
{
//    SerialPrintF(P("<====Tail :"));
//    SerialPrintU32(addr);
//    SerialPrintF(P("====>"));
    flashRead(addr, buf, sizeof(payload_t));
}

void memWriter(uint32_t addr, uint8_t *buf, uint16_t len)
{
//  SerialPrintF(P("<====Head :"));
//    SerialPrintU32(addr);
//    SerialPrintF(P("====>"));
    flashWrite(addr,buf,sizeof(payload_t));
}
void memEraser(uint32_t addr, uint16_t len)
{
//  uint8_t *flashPtr = (uint8_t*)&dataStruct + (uint16_t)addr;
//  memset(flashPtr, 0, len);
    flashEraseSector(addr);
    uint32_t curPage = addr;
//    flashDumpPage(curPage, pageBuf);
}

void memPtrReader(struct memqPtr_t *ptr)
{
#if defined(SHOW_DEBUG)
    SerialPrintln("MemQPtr>R>");
#endif
//    memcpy(ptr, &ring, sizeof(ptr_t));
//  ringObj.readPacket((byte *)ptr);
    FRAMRead(FRAM_write_ptr, (uint8_t *)ptr, sizeof(struct memqPtr_t));
}

void memPtrWriter(struct memqPtr_t *ptr)
{
#if defined(SHOW_DEBUG)
  SerialPrintln("MemQPtr>W>");
#endif
//   ringObj.savePacket((byte *)ptr);
//  memcpy(&ring, ptr, sizeof(ptr_t));
  FRAMWrite(FRAM_write_ptr, (uint8_t *)ptr, sizeof(struct memqPtr_t));
}

void printBuffer(uint8_t *buf, uint8_t len)
{
   SerialPrint("\r\n");
   uint8_t i;
  for ( i = 0; i < len; i++)
  {
      SerialPrintU8(buf[i]); SerialPrint(" ");
  }
  SerialPrint("\r\n");
}


//uint8_t getSerialCmd()
//{
//    SerialPrint("Input Cmd :");
//  while (!Serial.available())
//  {
//    delay(10);
//  }
//  int cmd = Serial.parseInt();
//  Serial.println(cmd);
//  return cmd;
//}

void generatePld(payload_t *pld)
{
//  static uint32_t data;
//  uint8_t i;
//  for(i = 0; i< 8; i++)
//  {
//    pld ->value[i] = ++data;
//  }
}
void printPld(payload_t *pld)
{
//    SerialPrint("Payload :");
//    uint8_t i;
//  for(i = 0; i<8 ; i++)
//  {
//    SerialPrintU8(pld -> value[i]);SerialPrint("  ");
//  }
//  SerialPrintln("");
}

void disableOthersOnbus()
{
  SerialPrintln("Disable others on bus");
}

void enableOthersOnbus()
{
  SerialPrintln("Enable others on bus");
}





void framRead(uint32_t addr, uint8_t *buf, uint16_t len)
{
  SerialPrint("FRAM Reading : ");SerialPrintlnU32(addr);
  unsigned int i = 0;
  uint8_t *ptr = (uint8_t*) buf;
  uint8_t *writePtr = (uint8_t*)addr;
  for (i = 0; i < len; i++)
  {
      *ptr++ = *writePtr++;
  }
//  ptr = (uint8_t*) addrPtr;
//  nrfDebugBuffer(ptr, sizeof(addr_t));
}

void framUpdate(uint32_t addr, uint8_t *buf, uint16_t len)
{

    SerialPrint("FRAM Updating : ");SerialPrintlnU32(addr);
  unsigned int i = 0;
  uint8_t *ptr = buf;
  uint8_t *writePtr = (uint8_t*)addr;

  SYSCFG0 = FRWPPW | PFWP;
  for (i = 0; i < len; i++)
  {
      *writePtr++ = *ptr++;
  }
  SYSCFG0 = FRWPPW | PFWP | DFWP;

//  ptr = (uint8_t*) addrPtr;

}

void flashPinSetup()
{
    flashSetPin(&P3OUT, FLASH_CS);
#if defined(BOARD_MSP430FR_V010)
    P3DIR |= (1<<FLASH_HOLD);
#endif
    FLASH_HOLD_PORT |= (1<<FLASH_HOLD);
}
