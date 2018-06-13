/*
 * File:		USB_device.c
 * Purpose:		Main process
 *
 */
#include "common.h"     /* Common definitions */
#include "USB_CDC.h"    /* USB DCD support */
#include "USB_Reg.h"    /* USB regulator */
#include "USB_DCD.h"    /* DCD support */
#include "psptypes.h"
#include "MAX3353.h"
#include "Settings.h"

#if ( (defined(TWR_K40X256) | defined (TWR_K53N512)) )
#include "i2c.h"
#else /* TWR_K60N512 */
#include "i2c_1.h"
#endif

#if ( defined(TWR_K40X256) | defined (TWR_K53N512) )
    #define ENABLE_USB_5V   GPIOB_PSOR|=(1<<9);
    #define DISABLE_USB_5V  GPIOB_PCOR|=(1<<9);
#else /* TWR_K60N512 */
    #define ENABLE_USB_5V   GPIOB_PSOR|=(1<<8);
    #define DISABLE_USB_5V  GPIOB_PCOR|=(1<<8);
#endif

/* Extern Variables */
extern UINT8 gu8USB_Flags; 
extern UINT8 gu8EP3_OUT_ODD_Buffer[];
extern tBDT tBDTtable[];
extern uint_8 gu8MAX3353StatusRegister;

/* Global Variales */
uint_8  gu8ISR_Flags=0;


UINT8 gu8DebugTrace[1000];
UINT8 *pu8trace;



/********************************************************************/
void PortB_ISR(void)
{
    uint_32 u32Value;
    u32Value=PORTB_ISFR;
    PORTB_ISFR=u32Value;
    MAX3353_ISR();
}

/********************************************************************/
void PortA_ISR(void)
{
    uint_32 u32Value;
    u32Value=PORTA_ISFR;
    PORTA_ISFR=u32Value;
    MAX3353_ISR();
}

/********************************************************************/
void port_config (void)
{
    extern uint32 __VECTOR_RAM[];           //Get vector table that was copied to RAM

    /* Turn on all port clocks */
    SIM_SCGC5|= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

    /* NVIC Configuration */

    #if ( defined(TWR_K40X256) | defined (TWR_K53N512) )
    __VECTOR_RAM[104]=(uint_32)PortB_ISR;   //replace ISR
    NVICICER2|=(1<<24);                     //Clear any pending interrupts
    NVICISER2|=(1<<24);                     //Enable interrupts

    PORTB_PCR9=(0|PORT_PCR_MUX(1));         // configure pin as GPIO
    GPIOB_PDDR|=(1<<9);                     // set as output        
    PORTB_PCR7=(0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x0A));   
    
    #else  /* TWR_K60N512 */  
    __VECTOR_RAM[103]=(uint_32)PortA_ISR;   //replace ISR
    NVICICER2|=(1<<23);                     //Clear any pending interrupts
    NVICISER2|=(1<<23);                     //Enable interrupts

    PORTB_PCR8=(0|PORT_PCR_MUX(1));         // configure pin as GPIO
    GPIOB_PDDR|=(1<<8);                     // set as output
    PORTA_PCR26=(0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x0A));       
    #endif
}



/********************************************************************/
void main (void)
{   
    pu8trace=gu8DebugTrace;
    
    // Making sure tat Regulator is enables
    USB_REG_SET_ENABLE;
    USB_REG_CLEAR_STDBY;

    port_config();
    init_I2C();

    DISABLE_USB_5V;                     // Disable USB 5v (for Host support)

    if(MAX3353_Init())
        printf("\nMAX3353 not Present\n");
    else
        printf("\nMAX3353 Present\n");

    
    CDC_Init();     // USB CDC Initialization
    DCD_Init();     // Enabling DCD

    /* Here we activate te DCD start sequence. This section must be triggered 
      with the VBUs detection pin, but in this case will work once after
      the board is powered in the Kinetis USB connector */    

    printf("\n********** DCD/USB Device Module Testing **********\n");
    
    while(1)
    {
        // USB CDC service routine
        CDC_Engine();
        
        // DCD detection service
        (void)DCD_Service();

        // MAX3353 service
        if(FLAG_CHK(_MAX3353,gu8ISR_Flags))
        {
            if(gu8MAX3353StatusRegister & VBUS_HIGH_EVENT)
            {    
                USBDCD_CONTROL=USBDCD_CONTROL_IE_MASK  | USBDCD_CONTROL_IACK_MASK;          
                FLAG_SET(USBDCD_CONTROL_START_SHIFT,USBDCD_CONTROL);
                printf("\nVBUS is Up");
            }
            if(gu8MAX3353StatusRegister & VBUS_LOW_EVENT)
            {
                USB_DISABLE_PULLUP;
                printf("\nVBUS is Down");
            }
            FLAG_CLR(_MAX3353,gu8ISR_Flags);
        }

        // If data transfer arrives
        if(FLAG_CHK(EP_OUT,gu8USB_Flags))
        {
            (void)USB_EP_OUT_SizeCheck(EP_OUT);         
            usbEP_Reset(EP_OUT);
            usbSIE_CONTROL(EP_OUT);
            FLAG_CLR(EP_OUT,gu8USB_Flags);

            // Send it back to the PC
            EP_IN_Transfer(EP2,CDC_OUTPointer,1);
        }        
    }
}
