/*
 * File:		lptmr_counter.c
 * Purpose:		Time Counter Examples
 *
 */

#include "lptmr.h"

/*
 *  Counting example using the LPO clock.
 *
 *  Sets compare value to 5000. Thus when using the 1Khz LPO clock,
 *   it will take 5 seconds for the Timer Compare Flag to be set.
 *
 */
void lptmr_time_counter()
{
  int compare_value=5000;  //value must be less than 0xFFFF or 65535

  printf("\n\n****************************\n");
  printf("LPTMR Time Counting Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock and bypass prescale

  printf("LPTMR using LPO clock with no prescale, and compare value=5000 (5 seconds)\n");
  printf("Press a key to start counter\n");
  in_char(); //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR with default settings

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("Waited for %d counts\n",compare_value);
  printf("End of Time Counting Example\n");
  printf("****************************\n\n");
}

/*
 *  Counting example using the LPO clock and the prescale feature
 *
 *  Sets compare value to 250. Thus when using the 1Khz LPO clock with
 *    LPTMR0_PSR[PRESCALE]=0x4, it will take 8 seconds for Timer Compare Flag
 *    to be set. (1Khz clock/32=31.25Hz clock)
 *
 *  If prescaler was not used, then timer would only wait .25 seconds.
 *
 */
void lptmr_prescale()
{
  int compare_value=250;  //value must be less than 0xFFFF or 65535

  printf("\n\n****************************\n");
  printf("LPTMR Time Counting Example with Prescaler\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PRESCALE(0x4);  //Use LPO clock and divide by 32

  printf("LPTMR using LPO clock with PRESCALE=4 and compare value=250 (8 seconds)\n");
  printf("Press a key to start counter\n");
  in_char();  //wait for keyboard press

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPTMR with default settings

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("Waited for %d counts\n",compare_value);
  printf("End of Time Counting Example with Prescale\n");
  printf("****************************\n\n");
}