/*
 * File:		lptmr_interrupt.c
 * Purpose:		Example using interrupts with the LPTMR
 *                      Vector 101 redefined in isr.h
 *
 */


#include "lptmr.h"

char LPTMR_INTERRUPT;  //Global flag variable

/*
 * Timer will trigger interrupt after 5 seconds
 */
void lptmr_interrupt(void)
{
  int compare_value=5000;  //value must be less than 0xFFFF
  LPTMR_INTERRUPT=0; //Clear global variable

  //Reset LPTMR module
  lptmr_clear_registers();

  printf("\n\n****************************\n");
  printf("LPTMR Interrupt Example\n");

  /* Enable LPT Interrupt in NVIC*/
  enable_irq(85); //LPTMR Vector is 101. IRQ# is 101-16=85

  /* Configure LPT */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock and bypass prescale
  LPTMR0_CSR=LPTMR_CSR_TIE_MASK;  //Enable LPT interrupt

  printf("LPTMR using LPO clock with no prescale, and compare value=5000 (5 seconds)\n");
  printf("Press a key to start counter\n");
  in_char(); //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR and start counting

  printf("Counting...\n\n");
  /* Wait for the global variable to be set in LPTMR ISR */
  while(LPTMR_INTERRUPT==0)
  {}

  printf("Timer should have waited for 5 seconds\n");

  LPTMR0_CSR&=~LPTMR_CSR_TEN_MASK; //Turn off LPT to avoid more interrupts
  printf("End of LPTMR Interrupt Example\n");
  printf("****************************\n\n");

  //Reset LPTMR module
  lptmr_clear_registers();
}

void lptmr_isr(void)
{
  LPTMR0_CSR|=LPTMR_CSR_TCF_MASK;  //Clear LPT Compare flag
  LPTMR_INTERRUPT=1;  //Set global variable
  printf("\n\nIn LPT ISR!\n\n");
}