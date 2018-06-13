/*
 * File:		lptmr_clocksource.c
 * Purpose:		Examples of using different clock sources
 *
 */

#include "lptmr.h"

/*
 * Internal Reference Clock (PSC=0x0)
 *   The Internal Reference Clock can come from two clock sources.
 *   If MCG_C2[IRCS]=0, then uses slow internal clock (32kHz)
 *   If MCG_C2[IRCS]=1, then uses fast internal clock (2Mhz)
 *
 *  This example uses fast internal clock. It is pre-scaled to wait for 4 seconds.
 *    Because of trim values, it may be slightly faster or slower than this.
 */
void lptmr_internal_ref_input()
{
  unsigned int compare_value=15625; //4 seconds with prescale=8 and 2Mhz fast clock

  printf("\n\n****************************\n");
  printf("Internal Reference Clock Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Ensure Internal Reference Clock is Enabled */
  MCG_C1|=MCG_C1_IRCLKEN_MASK;

  //Enable fast internal ref clock by setting MCG_C2[IRCS]=1
  //If wanted to use 32Khz slow mode, set MCG_C2[IRCS]=0 instead
  MCG_C2|=MCG_C2_IRCS_MASK;

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x0)|LPTMR_PSR_PRESCALE(0x8);  //Use internal clock prescaled by 512

  printf("LPTMR using fast internal ref clock with PRESCALE=0x8, and compare value=15625 (4 seconds)\n");
  printf("Press a key to start counter\n");
  in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("4 seconds should have passed\n");
  printf("End of Internal reference Clock Source Example\n");
  printf("****************************\n\n");
}

/*
 * LPO Clock Example (PSC=0x1)
 *
 *  Test is pre-scaled to wait for 10 seconds. In reality it might be slightly off because
 *   of the trim values for the LPO clock.
 */
void lptmr_lpo_input()
{
  unsigned int compare_value=4000; //4 second delay with the 1khz LPO clock

  printf("\n\n****************************\n");
  printf("LPO Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use LPO clock with bypass enabled

  printf("LPTMR using LPO clock with no prescale, and compare value=4000 (4 seconds)\n");
  printf("Press a key to start counter\n");
  in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("4 seconds should have passed\n");
  printf("End of LPO Clock Source Example\n");
  printf("****************************\n\n");
}

/*
 * 32kHz Input Clock Test (PSC=0x2)
 *   The 32kHz clock (ERCLK32K) can come from two clock sources.
 *   If SOPT1[OSC32KSEL]=1, then it uses 32kHz RTC crystal connected to XTAL32
 *      This is what the code below tests
 *   If SOPT1[OSC32KSEL]=0, then it uses 32kHz System oscillator, and reguires
 *      that the main system clock be a 32kHz crystal. The tower board does not
 *      support that feature.
 *
 *  Test is pre-scaled to wait for 8 seconds.
 */
void lptmr_32khz_input()
{
  unsigned int compare_value=32768; //4 second delay with prescale=1

  printf("\n\n****************************\n");
  printf("32 Khz Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /*
   * Configure to use 32Khz clock from RTC clock
   */
  printf("Test using RTC OSC\n");
  SIM_SCGC6|=SIM_SCGC6_RTC_MASK; //Enable RTC registers
  RTC_CR|=RTC_CR_OSCE_MASK;      //Turn on RTC oscillator
  SIM_SOPT1|=SIM_SOPT1_OSC32KSEL_MASK;  //Select RTC OSC as source for ERCLK32K

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x2)|LPTMR_PSR_PRESCALE(0x1);  //Use 32khz clock (ERCLK32K) and divide source by 4

  printf("LPTMR using 32Khz clock with PRESCALE=0x1, and compare value=32768 (4 seconds)\n");
  printf("Press a key to start counter\n");
  in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("4 seconds should have passed\n");
  printf("End of 32 Khz Clock Source Example\n");
  printf("****************************\n\n");
}


/*
 * External Reference Clock Example(PSC=0x3)
 *
 * TWR-K60N512 uses a 50MHz external clock
 * TWR-K40X256 uses a 8MHz external clock
 *
 *  Test is pre-scaled to wait for 10 seconds in both cases by adjusting
 *    the compare value.
 */
void lptmr_external_clk_input()
{
  unsigned int compare_value;

  printf("\n\n****************************\n");
  printf("External Clock Source Example\n");

  //Reset LPTMR module
  lptmr_clear_registers();

  /* Turn on external reference clock */
  MCG_C2&=~MCG_C2_EREFS_MASK;     //allow extal to drive
  OSC_CR |= OSC_CR_ERCLKEN_MASK;  //  selects EXTAL to drive  XOSCxERCLK

  //Determine compare value based on which board is being used
#if (defined(K60_CLK) | defined(K53_CLK))
  compare_value=7630; //~10 seconds with prescale=0xF and 50MHz clock
#else
  compare_value=1220; //~10 seconds with prescale=0xF and 8MHz clock
#endif

  /* Configure LPTMR */
  LPTMR0_CMR=LPTMR_CMR_COMPARE(compare_value);  //Set compare value
  LPTMR0_PSR=LPTMR_PSR_PCS(0x3)|LPTMR_PSR_PRESCALE(0xF);  //Use external clock divided by 65536

  printf("LPTMR using external clock with PRESCALE=0xF, and compare value=%d (10 seconds)\n",compare_value);
  printf("Press a key to start counter\n");
  in_char();

  LPTMR0_CSR|=LPTMR_CSR_TEN_MASK; //Turn on LPT with default settings

  printf("Counting...\n\n");

  //Wait for Timer Compare Flag to be set
  while((LPTMR0_CSR&LPTMR_CSR_TCF_MASK)==0)
  {
    //This may not get proper counter data if the CNR is read at the same time it is incremented
    //printf("Current value of counter register CNR is %d\n",LPTMR0_CNR);
  }

  printf("Timer should have waited for 10 seconds\n");
  printf("End of External Clock Example\n");
  printf("****************************\n\n");
}
