#include "MAX3353.h"

/* Global variables */
uint_8 gu8MAX3353StatusRegister=0;

/* Extern Variables */
extern uint_8  gu8ISR_Flags;


/********************************************************************/
uint_8 MAX3353_Init(void)
{
    if(MAX3353_CheckifPresent())
        return(MAX3353_NOT_PRESENT);

    // Enable Charge pump for VBUs detection
    MAX3353_Write_Register(MAX3353_REG_CTRL_2,0x80);

    // Set Rising edge for VBUS detection
    MAX3353_Write_Register(MAX3353_REG_INT_EDGE,VBUS_VALID_ED_MASK);

    // Activate ID (GND and float) & SESSION Interrupts
    MAX3353_Write_Register(MAX3353_REG_INT_MASK,ID_FLOAT_EN_MASK|ID_GND_EN_MASK\
                           |VBUS_VALID_EN_MASK|SESSION_SESSEND_EN_MASK); 
    
    return(MAX3353_OK);
}


/********************************************************************/
uint_8 MAX3353_Read_Register(uint_8 u8Register)
{
    return(I2C_ReadRegister(MAX3353_I2C_ADDRESS,u8Register));
}

/********************************************************************/
void MAX3353_Write_Register(uint_8 u8Register, uint_8 u8Value)
{
    I2C_WriteRegister(MAX3353_I2C_ADDRESS,u8Register,u8Value);
}

/********************************************************************/
void MAX3353_ISR(void)
{
    // Clear the Interrupt pending flag
    gu8MAX3353StatusRegister=I2C_ReadRegister(MAX3353_I2C_ADDRESS,MAX3353_REG_INT_LATCH);

    // Set MAX3353 ISR flag
    FLAG_SET(_MAX3353,gu8ISR_Flags);
}

uint_32 u32ID=0;

/********************************************************************/
uint_8 MAX3353_CheckifPresent(void)
{

    uint_8  u8Counter;
    
    for(u8Counter=0;u8Counter<4;u8Counter++)
    {
        u32ID=u32ID<<8;
        u32ID|=I2C_ReadRegister(MAX3353_I2C_ADDRESS,u8Counter);
    }
    if(u32ID!=MAX3353_MID)
        return(MAX3353_NOT_PRESENT);

    u32ID=0;
    for(u8Counter=4;u8Counter<8;u8Counter++)
    {
        u32ID=u32ID<<8;
        u32ID|=I2C_ReadRegister(MAX3353_I2C_ADDRESS,u8Counter);
    }
    if(u32ID!=MAX3353_PID)
        return(MAX3353_NOT_PRESENT);

    return(MAX3353_OK);
}
