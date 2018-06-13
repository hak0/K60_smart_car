#include "RingBuffer.h"

/* Pointers */
volatile UINT8 *OUT_StartAddress;
volatile UINT8 *OUT_EndAddress;
volatile UINT8 *OUT_UsbPointer;
volatile UINT8 *OUT_SciPointer;

/* Variables */
volatile UINT8 gu8BufferMaxSize;
volatile UINT8 gu8BufferOverFlow;


/********************************************************/
void Buffer_Init(UINT8* pu8BufferPointer ,UINT8 u8MaxSize)
{
    /* Buffer Initialization */
    OUT_EndAddress=pu8BufferPointer+u8MaxSize-1;
    OUT_StartAddress=pu8BufferPointer;
    gu8BufferMaxSize=u8MaxSize;
    OUT_UsbPointer=pu8BufferPointer;
    OUT_SciPointer=pu8BufferPointer;
    gu8BufferOverFlow=0;
}


/********************************************************/
UINT8 Buffer_Request(UINT8* pu8DataPointer ,UINT16 u8RequestSize)
{
    UINT8 u8FreeSpace;
    
    // Check for OverFlow
    
    if(gu8BufferOverFlow)
        return(NOT_ENOUGH_SPACE);
    
    
    // Calculate Free Space
    if(OUT_UsbPointer < OUT_SciPointer)
        u8FreeSpace=(UINT8)(OUT_SciPointer-OUT_UsbPointer);
    
    else
        u8FreeSpace=gu8BufferMaxSize-(OUT_UsbPointer-OUT_SciPointer);
    
    // Validate requested size
    if(u8FreeSpace<u8RequestSize)
        return(NOT_ENOUGH_SPACE);
    
    
    if(u8FreeSpace==u8RequestSize)
        gu8BufferOverFlow=1;
    
    // Buffer Copy
    while(u8RequestSize--)
    {
        *OUT_UsbPointer=*pu8DataPointer;
        OUT_UsbPointer++;
        pu8DataPointer++;
        if(OUT_UsbPointer > OUT_EndAddress)
        {
            OUT_UsbPointer=OUT_StartAddress;
        }
    }
    return(OK);
}