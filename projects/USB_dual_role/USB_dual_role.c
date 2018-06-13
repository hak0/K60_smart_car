/*
 * File:		USB_dual_role.c
 * Purpose:		Main process
 *
 */
#include "common.h"
#include "psptypes.h"
#include "MAX3353.h"
#include "Settings.h"
#include "USB_device.h"
#include "hidmouse.h"
#include "pmc.h"

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



enum
{
    USB_DEVICE_MODE,
    USB_HOST_MODE,
    USB_DEVICE_MODE_INIT,
    USB_HOST_MODE_INIT,
};

/* Global Variales */
uint_8  gu8ISR_Flags=0;
uint_8  gu8USB_Mode=USB_DEVICE_MODE_INIT;


/* Extern Variables */
extern uint_8 gu8MAX3353StatusRegister;



/* Prototypes */
void USB_State_Machine(void);

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
void Display_Intro(void)
{
    printf("\n\n*************************************************************");
    printf("\n*           USB Dual-Role example using MAX3353             *");
    printf("\n*************************************************************");
    printf("\n\nTWR-Serial board Jumpers");
    printf("\nJ16 -> 1-2   5-6");
    printf("\nJ11 -> 5-6");
    printf("\n*************************************************************\n");
}


/********************************************************************/
void main (void)
{
    Display_Intro();
    init_I2C();
    port_config();
    DISABLE_USB_5V;                     // Disable USB 5v (for Host support)


    MAX3353_ISR();
    
    if(MAX3353_Init())
        printf("\nMAX3353 not Present\n");
    else
        printf("\nMAX3353 Present\n");
      

    while(1)
    {

        /*** Check for changes ****/
        if(FLAG_CHK(_MAX3353,gu8ISR_Flags))
        {
            FLAG_CLR(_MAX3353,gu8ISR_Flags);
            if(gu8MAX3353StatusRegister & ID_GND_EVENT)
            {

                printf("\nID-GND   --> Changing to HOST mode (Enabling 5V)");
                gu8USB_Mode=USB_HOST_MODE_INIT;
            }    
            if(gu8MAX3353StatusRegister & ID_FLOAT_EVENT)
            {
                printf("\nID-FLOAT --> Changing to DEVICE mode (Disabling 5V)");
                gu8USB_Mode=USB_DEVICE_MODE_INIT;
            }    
            if(gu8MAX3353StatusRegister & VBUS_HIGH_EVENT)
            {    
                printf("\nVBUS is Up");
            }

            if(gu8MAX3353StatusRegister & VBUS_LOW_EVENT)
            {    
                printf("\nVBUS is Down");
                if(gu8USB_Mode==USB_DEVICE_MODE)
                    gu8USB_Mode=USB_DEVICE_MODE_INIT;    
                //enter_vlps();   // Enter VLPS
                
                //mcg_pbe_2_pee(); // Exit VLPS
                // Low power
            }
        
        }      

        /* USB Switch stacks */
        USB_State_Machine();

      

    } 
}


/**************************************************/
void USB_State_Machine(void)
{
        /*** USB Stage ***/      
        if(gu8USB_Mode==USB_DEVICE_MODE_INIT)
        {
            DISABLE_USB_5V;
            USB_Device_Init();
            gu8USB_Mode=USB_DEVICE_MODE;
        }
        if(gu8USB_Mode==USB_DEVICE_MODE)
        {
            USB_Device_Task();
        }
        
        if(gu8USB_Mode==USB_HOST_MODE_INIT)
        {
            USB_Host_Free_Memory();
            USB_Host_Init();
            gu8USB_Mode=USB_HOST_MODE;
            ENABLE_USB_5V;        
        }
        if(gu8USB_Mode==USB_HOST_MODE)
        {
            USB_Host_Task();
        }      
}
