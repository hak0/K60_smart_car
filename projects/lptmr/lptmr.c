/*
 * File:		lptmr.c
 * Purpose:		Low Power Timer (LPTMR) examples
 *
 */

#include "lptmr.h"

/********************************************************************/
void main (void)
{
  printf("\n\n****************************\n");
  printf("LPTMR Examples\n");
  printf("****************************\n\n");

  //Enable Port clocks
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

  /* Enable LPT Module */
  SIM_SCGC5|=SIM_SCGC5_LPTIMER_MASK;

  /* Time counting using LPO clock */
  lptmr_time_counter();

  /* Time counting using LPO clock with Prescaling */
  lptmr_prescale();

  /* Time counting using different clock sources */
  lptmr_internal_ref_input();
  lptmr_lpo_input();
  lptmr_32khz_input();
  lptmr_external_clk_input();

  /* LPTMR Interrupt Example */
  lptmr_interrupt();

  /* Basic Pulse Counting */
  lptmr_pulse_counter(LPTMR_ALT1);
  lptmr_pulse_counter(LPTMR_ALT2);

  printf("\n****************************\n");
  printf("End of LPTMR Examples\n");
  printf("****************************\n");
}
/********************************************************************/

/*
 * Zero out all registers.
 *
 */
void lptmr_clear_registers()
{
  LPTMR0_CSR=0x00;
  LPTMR0_PSR=0x00;
  LPTMR0_CMR=0x00;
}