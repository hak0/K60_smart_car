#include "USB_DCD.h"

/* Global Variables */
UINT8 gu8InterruptFlags=0;
extern UINT32 __VECTOR_RAM[]; //Get vector table that was copied to RAM


/***************************************************************/
void DCD_Init(void)
{
    // SIM Configuration   
    SIM_SCGC6|=(SIM_SCGC6_USBDCD_MASK);             // USBDCD Clock Gating

    
    // Enable DCD Interrupt in NVIC
    __VECTOR_RAM[90]=(UINT32)DCD_ISR;       // replace ISR
    //NVICICER2|=(1<<10);                     // Clear any pending interrupts on DCD
    NVICISER2|=(1<<10);                     // Enable interrupts from DCD module
    //NVICICPR2|=(1<<10);                     // Clear any pending interrupts on DCD
    
    USBDCD_CLOCK=(DCD_TIME_BASE<<2)|1;
}

/***************************************************************/
UINT8 DCD_Service(void)
{
    //gu8InterruptFlags=0;
    UINT32 u8Error;
        
        // DCD results
        if(FLAG_CHK(DCD_Flag,gu8InterruptFlags))
        {
            u8Error=DCD_GetChargerType();

            if((u8Error&0xF0))
                printf("Oooooops DCD Error");
            else
            {     
                if((u8Error&0x0F)==STANDARD_HOST)
                    printf("\nConnected to a Standard Host");
                if((u8Error&0x0F)==CHARGING_HOST)
                    printf("\nConnected to a Charging Host");
                if((u8Error&0x0F)==DEDICATED_CHARGER)
                    printf("\nConnected to a Dedicated Charger");
            }
            //DCD_ResetModule;  
            FLAG_CLR(DCD_Flag,gu8InterruptFlags);
            DCD_ResetModule;
            return(1);
        }
        return(0);
}

/***************************************************************/
void DCD_Signal_Testing(void)
{

  // Active 
    if(USBDCD_STATUS & USBDCD_STATUS_ACTIVE_MASK)
        FLAG_SET(DCD_ACTIVE,DEBUG_PORT);
    else
        FLAG_CLR(DCD_ACTIVE,DEBUG_PORT);
    


    // SEQ_STAT0
    if(FLAG_CHK(USBDCD_STATUS_SEQ_STAT0_SHIFT,USBDCD_STATUS))
        FLAG_SET(DCD_SEQ_STAT0,DEBUG_PORT);
    else
        FLAG_CLR(DCD_SEQ_STAT0,DEBUG_PORT);
    
    // SEQ_STAT1
    if(FLAG_CHK(USBDCD_STATUS_SEQ_STAT1_SHIFT,USBDCD_STATUS))
        FLAG_SET(DCD_SEQ_STAT1,DEBUG_PORT);
    else
        FLAG_CLR(DCD_SEQ_STAT1,DEBUG_PORT);
    


    // TimeOut
    if(USBDCD_STATUS & USBDCD_STATUS_TO_MASK)
        FLAG_SET(DCD_TO,DEBUG_PORT);
    else
        FLAG_CLR(DCD_TO,DEBUG_PORT);


    
    // Error
    if(USBDCD_STATUS & USBDCD_STATUS_ERR_MASK)
        FLAG_SET(DCD_ERR,DEBUG_PORT);
    else
        FLAG_CLR(DCD_ERR,DEBUG_PORT);
    
    
    
    // SEQ_RES0
    if(FLAG_CHK(USBDCD_STATUS_SEQ_RES0_SHIFT,USBDCD_STATUS))
        FLAG_SET(DCD_SEQ_RES0,DEBUG_PORT);
    else
        FLAG_CLR(DCD_SEQ_RES0,DEBUG_PORT);
    
    // SEQ_RES1
    if(FLAG_CHK(USBDCD_STATUS_SEQ_RES1_SHIFT,USBDCD_STATUS))
         FLAG_SET(DCD_SEQ_RES1,DEBUG_PORT);
    else
        FLAG_CLR(DCD_SEQ_RES1,DEBUG_PORT);
    
    if((USBDCD_TIMER0 & 0x00000FFF) == 500)
        FLAG_SET(DCD_DEBUG,DEBUG_PORT);
    
}

/***************************************************************/
UINT8 DCD_Out_of_reset_test(void)
{
    UINT8 u8Error=0;
            
    if(USBDCD_CONTROL != USBDCD_CONTROL_RESET_VALUE)
        u8Error|=1<<0;
    if(USBDCD_CLOCK != USBDCD_CLOCK_RESET_VALUE)
        u8Error|=1<<1;
    if(USBDCD_STATUS != USBDCD_STATUS_RESET_VALUE)
        u8Error|=1<<2;
    if(USBDCD_TIMER0 != USBDCD_TIMER0_RESET_VALUE)
        u8Error|=1<<3;
    if(USBDCD_TIMER1 != USBDCD_TIMER1_RESET_VALUE)
        u8Error|=1<<4;
    if(USBDCD_TIMER2 != USBDCD_TIMER2_RESET_VALUE)
        u8Error|=1<<5;
    
    return(u8Error);
}

/***************************************************************/
UINT8 DCD_RegisterAccess_byte(void)
{
}

/***************************************************************/
UINT8 DCD_RegisterAccess_word(void)
{
}





/***************************************************************/
UINT8 DCD_GetChargerType(void)
{
    UINT8 u8ChargerType;
    
    u8ChargerType = (UINT8)((USBDCD_STATUS & USBDCD_STATUS_SEQ_RES_MASK)>>16);
    
    u8ChargerType|= (UINT8)((USBDCD_STATUS & USBDCD_STATUS_FLAGS_MASK)>>16);

    return(u8ChargerType);
}

/***************************************************************/
UINT8 DCD_RegisterAccess_long(void)
{
    UINT8 u8Error=0;
    
    // Testing DCD Control Register
    USBDCD_CONTROL=0x00000000;
    if(USBDCD_CONTROL != 0x00000000)
        u8Error|=1<<0;
    USBDCD_CONTROL=0x00010000;
    if(USBDCD_CONTROL != 0x00010000)
        u8Error|=1<<0;
    
    // Testing Clock Register
    USBDCD_CLOCK=0x00000000;
    if(USBDCD_CLOCK != 0x00000000)
        u8Error|=1<<1;
    USBDCD_CLOCK=0x00000FFD;
    if(USBDCD_CLOCK != 0x00000FFD)
        u8Error|=1<<1;
    
    // Testing timer0 Register
    USBDCD_TIMER0=0x00000000;
    if(USBDCD_TIMER0 != 0x00000000)
        u8Error|=1<<2;
    USBDCD_TIMER0=0x03FF0000;
    if(USBDCD_TIMER0 != 0x03FF0000)
        u8Error|=1<<2;
    
    // Testing Timer1 Register
    USBDCD_TIMER1=0x00000000;
    if(USBDCD_TIMER1 != 0x00000000)
        u8Error|=1<<3;
    USBDCD_TIMER1=0x03FF03FF;
    if(USBDCD_TIMER1 != 0x03FF03FF)
        u8Error|=1<<3;
                                   
    // Testing Timer2 Register
    USBDCD_TIMER2=0x00000000;
    if(USBDCD_TIMER2 != 0x00000000)
        u8Error|=1<<4;
    USBDCD_TIMER2=0x03FF000F;
    if(USBDCD_TIMER2 != 0x03FF000F)
        u8Error|=1<<4;
    
    return(u8Error);
}

extern UINT8 *pu8trace;

/***************************************************************/
void DCD_ISR(void)
{
    USBDCD_CONTROL|= USBDCD_CONTROL_IACK_MASK;
    
    if((USBDCD_STATUS&0x000C0000) == 0x00080000)
        USB_ENABLE_PULLUP;

    if((!(USBDCD_STATUS & 0x00400000)) || (USBDCD_STATUS & 0x00300000))
        FLAG_SET(DCD_Flag,gu8InterruptFlags);        
}

   