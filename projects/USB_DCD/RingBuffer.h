#ifndef __CircBuffer__
#define __CircBuffer__

/* Includes */
#include "FslTypes.h"


/* Error Codes */
#define     OK                  0
#define     NOT_ENOUGH_SPACE    1


/* Extern variables */
volatile extern UINT8 *OUT_StartAddress;        
volatile extern UINT8 *OUT_EndAddress;          
volatile extern UINT8 *OUT_UsbPointer;          
volatile extern UINT8 *OUT_SciPointer;          
volatile extern UINT8 gu8BufferOverFlow;


/* Prototypes */
void Buffer_Init(UINT8* ,UINT8);
UINT8 Buffer_Request(UINT8* ,UINT16);


#endif /* __CircBuffer__*/