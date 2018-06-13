#include "misc.h"

/********************************************************************************
 *   delay: delay
 * Notes:
 *    -
 ********************************************************************************/
void delay(uint32 i)
{
  for(i;i;i--)  //delay
  {
  }
}

/********************************************************************************
 *   GPIO_Init: Initializes GPIO controlling LED
 * Notes:
 *    -
 ********************************************************************************/
void GPIO_Init(void)
{
  ENABLE_GPIO_CLOCKS;

  LED0_EN;
  LED1_EN;
  
#if ((defined(TWR_K60N512)) | (defined(TWR_K40X256)))    
  LED2_EN;
  LED3_EN;
#endif

  LEDs_On();
  LED_Dir_Out();
}

void LEDs_On(void)
{
#ifdef TWR_K60N512
  GPIOA_PDDR = (1<<10)|(1<<11)|(1<<28)|(1<<29);
#elif defined (TWR_K40X256)
  GPIOB_PDDR = (1<<11);
  GPIOC_PDDR = ((1<<7)|(1<<8)|(1<<9));
#else
  GPIOC_PDDR = ((1<<7)|(1<<8));
#endif
}

void LED_Dir_Out(void)
{
#ifdef TWR_K60N512
  GPIOA_PDOR &= ~((1<<10)|(1<<11)|(1<<28)|(1<<29));
#elif defined (TWR_K40X256)
  GPIOB_PDOR &= ~(1<<11);
  GPIOC_PDOR &= ~((1<<7)|(1<<8)|(1<<9));
#else
  GPIOC_PDOR &= ~((1<<7)|(1<<8));  
#endif
}