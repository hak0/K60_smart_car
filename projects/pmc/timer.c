// LPT Routine to provide programmable delays
#include "common.h"

void lpt_timer_setup(void)
{
  LPTMR0_PSR = LPTMR_PSR_PCS(1);
  return;
}

void lpt_time_delay_ms(int count_val)
{
  LPTMR0_CMR = count_val;
  LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;
  while (!(LPTMR0_CSR & LPTMR_CSR_TCF_MASK)) {}
  LPTMR0_CSR &= ~LPTMR_CSR_TEN_MASK;
  return;
}