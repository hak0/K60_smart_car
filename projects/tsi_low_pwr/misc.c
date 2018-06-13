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
  LED2_EN;
  LED3_EN;

  LEDs_On();
  LED_Dir_Out();
}

void LEDs_On(void)
{
#ifdef CPU_MK60N512VMD100
  GPIOA_PDDR |= (1<<10)|(1<<11)|(1<<28)|(1<<29);
#else
  GPIOB_PDDR |= (1<<11);
  GPIOC_PDDR |= ((1<<7)|(1<<8)|(1<<9));
#endif
}

void LEDs_Off(void)
{
#ifdef CPU_MK60N512VMD100
  GPIOA_PSOR |= (1<<10)|(1<<11)|(1<<28)|(1<<29);
#else  
  GPIOB_PSOR |= (1<<11);
  GPIOC_PSOR |= ((1<<7)|(1<<8)|(1<<9));
#endif
}

void LED_Dir_Out(void)
{
#ifdef CPU_MK60N512VMD100
  GPIOA_PDOR &= ~((1<<10)|(1<<11)|(1<<28)|(1<<29));
#else
  GPIOB_PDOR &= ~(1<<11);
  GPIOC_PDOR &= ~((1<<7)|(1<<8)|(1<<9));
#endif
}

void RTC_Init(void)
{
  SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL_MASK;
  SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;
  RTC_CR |= RTC_CR_OSCE_MASK;
  // RTC_TSR = 0x00000005;
}