#ifndef __CDC__
#define __CDC__

/* Includes */
#include"USB.h"
#include "FslTypes.h"
#include "RingBuffer.h"
//#include "SCI1.h" 
//#include "S08_Flash.h" 

extern UINT8 u8CDCState;

//#define CUSTOM_BASE_ADDRESS 0xC000  /* Base Address for Customizable settings */


#define CDC_INPointer   gu8EP2_IN_ODD_Buffer
#define CDC_OUTPointer  gu8EP3_OUT_ODD_Buffer

#define LoaderCounter CDC_OUT_Data[CDC_BUFFER_SIZE-1]


/* Defines */
#define CDC_BUFFER_SIZE 128
#define EP_OUT          3
#define EP_IN           2   



/* USB_CDC Definitions */
#define WAITING_FOR_ENUMERATION 0x00
#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22
#define LOADER_MODE             0xAA
#define GET_INTERFACE           0x0A
#define GET_STATUS              0x00
#define CLEAR_FEATURE           0x01
#define SET_FEATURE             0x03

/* TypeDefs */
typedef struct
{
    UINT32  DTERate;
    UINT8   CharFormat;
    UINT8   ParityType;
    UINT8   Databits;
}CDC_Line_Coding;


/* Extern variables */
extern UINT8 CDC_OUT_Data[];       
//extern UINT8 u8RecDataFlag;        
extern CDC_Line_Coding LineCoding; 


/* Prototypes */
void CDC_Init(void);
void CDC_Engine(void);
UINT8 CDC_InterfaceReq_Handler(void);
UINT32 LWordSwap(UINT32);
UINT8 CDC_Settings_Update(UINT8*,UINT8);


#endif /* __CDC__*/