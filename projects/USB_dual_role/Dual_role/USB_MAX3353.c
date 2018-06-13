#include "common.h"
#include "psptypes.h"
#include "MAX3353.h"
#include "Settings.h"

#ifdef TWR_K40X256
#include "i2c.h"
#else
#include "i2c_1.h"
#endif



#ifdef TWR_K40X256
    #define ENABLE_USB_5V   GPIOB_PSOR|=(1<<9);
    #define DISABLE_USB_5V  GPIOB_PCOR|=(1<<9);
#else
    #define ENABLE_USB_5V   GPIOB_PSOR|=(1<<8);
    #define DISABLE_USB_5V  GPIOB_PCOR|=(1<<8);
#endif


/* Global Variales */
uint_8  gu8ISR_Flags=0;

/* Extern Variables */
extern uint_8 gu8MAX3353StatusRegister;


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
    /* Turn on all port clocks */
    SIM_SCGC5|= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

    /* NVIC Configuration */
    extern uint32 __VECTOR_RAM[];           //Get vector table that was copied to RAM

    #ifdef TWR_K40X256
    __VECTOR_RAM[104]=(uint_32)PortB_ISR;   //replace ISR
    NVICICER2|=(1<<24);                     //Clear any pending interrupts
    NVICISER2|=(1<<24);                     //Enable interrupts

    PORTB_PCR9=(0|PORT_PCR_MUX(1));         // configure pin as GPIO
    GPIOB_PDDR|=(1<<9);                     // set as output        
    PORTB_PCR7=(0|PORT_PCR_MUX(1)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0x0A));   
    
    #else    
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
    printf("\n*           MAX3353 & MIC2026 Use for Kinetis               *");
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

    if(MAX3353_Init())
        printf("\nMAX3353 not Present\n");
    else
        printf("\nMAX3353 Present\n");
      
    while(1)
    {
        if(FLAG_CHK(_MAX3353,gu8ISR_Flags))
        {
            if(gu8MAX3353StatusRegister & ID_GND_EVENT)
            {
                ENABLE_USB_5V;
                printf("\nID-GND   --> Changing to HOST mode (Enabling 5V)");
            }    
            if(gu8MAX3353StatusRegister & ID_FLOAT_EVENT)
            {
                DISABLE_USB_5V;
                printf("\nID-FLOAT --> Changing to DEVICE mode (Disabling 5V)");
            }    
            if(gu8MAX3353StatusRegister & VBUS_HIGH_EVENT)
                printf("\nVBUS is Up");

            if(gu8MAX3353StatusRegister & VBUS_LOW_EVENT)
                printf("\nVBUS is Down");
        
            FLAG_CLR(_MAX3353,gu8ISR_Flags);
        }
    } 
}

