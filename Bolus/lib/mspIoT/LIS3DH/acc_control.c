#include "acc_control.h"

#define SAMPLE_IN_PACKET 8

//uint8_t ptrIndex=0;

uint8_t xyzBuffer[6];


void accIsr()
{
    struct bolus_t *headPtr = (union payload_t*)ramQHead();
    readAcc(headPtr);
}


//void *readAcc(struct bolus_t *ptr)
//{
//    struct bolus_t *curentHead = ptr;
//    uint8_t i = 0;
//    do
//    {
//        acc_get_xyz(xyzBuffer);
//        curentHead -> x[ptrIndex] = xyzBuffer[1];
//        curentHead -> y[ptrIndex] = xyzBuffer[3];
//        curentHead -> z[ptrIndex] = xyzBuffer[5];
//        ptrIndex++;
//        if( ptrIndex >= SAMPLE_IN_PACKET)
//        {
//            curentHead =  ramQUpdateHead();
//            ptrIndex = 0;
//        }
//    }while(++i <ACC_MAX_POINT);
//}


