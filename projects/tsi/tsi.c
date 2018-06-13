
#include "TSI.h"

extern uint32 __VECTOR_RAM[];

uint16  g16ElectrodeTouch[16] = {0};
uint16  g16ElectrodeBaseline[16] = {0};

uint32  g32DebounceCounter[16] = {DBOUNCE_COUNTS,DBOUNCE_COUNTS,DBOUNCE_COUNTS,DBOUNCE_COUNTS};

/********************************************************************************
 *   TSI_Init: Initializes TSI module
 * Notes:
 *    -
 ********************************************************************************/
void TSI_Init(void)
{
  SIM_SCGC5 |= (SIM_SCGC5_TSI_MASK); //Turn on clock to TSI module

#ifdef TWR_K60N512
  PORTA_PCR4 = PORT_PCR_MUX(0);      //Enable ALT0 for portA4
  PORTB_PCR2 = PORT_PCR_MUX(0);      //Enable ALT0 for portB2
  PORTB_PCR3 = PORT_PCR_MUX(0);      //Enable ALT0 for portB3
  PORTB_PCR16 = PORT_PCR_MUX(0);      //Enable ALT0 for portB16

#elif defined (TWR_K40X256)
  PORTB_PCR16 = PORT_PCR_MUX(0);      //Enable ALT0 for portB16
  PORTB_PCR17 = PORT_PCR_MUX(0);      //Enable ALT0 for portB17
  PORTB_PCR18 = PORT_PCR_MUX(0);      //Enable ALT0 for portB18
  PORTB_PCR19 = PORT_PCR_MUX(0);      //Enable ALT0 for portB19

#else /* TWR_K53N512 */
  PORTB_PCR17 = PORT_PCR_MUX(0);      //Enable ALT0 for portB17
  PORTB_PCR18 = PORT_PCR_MUX(0);      //Enable ALT0 for portB18
  
#endif

  TSI0_GENCS |= ((TSI_GENCS_NSCN(10))|(TSI_GENCS_PS(3)));
  TSI0_SCANC |= ((TSI_SCANC_EXTCHRG(3))|(TSI_SCANC_REFCHRG(31))|(TSI_SCANC_DELVOL(7))|(TSI_SCANC_SMOD(0))|(TSI_SCANC_AMPSC(0)));

#if ((defined(TWR_K60N512)) | (defined(TWR_K40X256)))  
  ELECTRODE_ENABLE_REG = ELECTRODE0_EN_MASK|ELECTRODE1_EN_MASK|ELECTRODE2_EN_MASK|ELECTRODE3_EN_MASK;
#else /* TWR_K53N512 */
  ELECTRODE_ENABLE_REG = ELECTRODE0_EN_MASK|ELECTRODE1_EN_MASK; //K53  
#endif

  TSI0_GENCS |= (TSI_GENCS_TSIEN_MASK);  //Enables TSI

  /* Init TSI interrupts */
  enable_irq(83);  //TSI Vector is 99. IRQ# is 99-16=83
  /***********************/

}

/********************************************************************************
 *   TSI_SelfCalibration: Simple auto calibration version
 * Notes:
 *    - Very simple, only sums a prefixed amount to the current baseline
 ********************************************************************************/
void TSI_SelfCalibration(void)
{
  TSI0_GENCS |= TSI_GENCS_SWTS_MASK;

  while(!TSI0_GENCS&TSI_GENCS_EOSF_MASK){};
  
  time_delay_ms(100);

  g16ElectrodeBaseline[ELECTRODE0] = ELECTRODE0_COUNT;
  ELECTRODE0_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE0]+ELECTRODE0_OVRRUN));
  g16ElectrodeTouch[ELECTRODE0] = g16ElectrodeBaseline[ELECTRODE0] + ELECTRODE0_TOUCH;

  g16ElectrodeBaseline[ELECTRODE1] = ELECTRODE1_COUNT;
  ELECTRODE1_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE1]+ELECTRODE1_OVRRUN));
  g16ElectrodeTouch[ELECTRODE1] = g16ElectrodeBaseline[ELECTRODE1] + ELECTRODE1_TOUCH;
  
#if ((defined(TWR_K60N512)) | (defined(TWR_K40X256)))  
  g16ElectrodeBaseline[ELECTRODE2] = ELECTRODE2_COUNT;
  ELECTRODE2_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE2]+ELECTRODE2_OVRRUN));
  g16ElectrodeTouch[ELECTRODE2] = g16ElectrodeBaseline[ELECTRODE2] + ELECTRODE2_TOUCH;

  g16ElectrodeBaseline[ELECTRODE3] = ELECTRODE3_COUNT;
  ELECTRODE3_OVERRUN = (uint32)((g16ElectrodeBaseline[ELECTRODE3]+ELECTRODE3_OVRRUN));
  g16ElectrodeTouch[ELECTRODE3] = g16ElectrodeBaseline[ELECTRODE3] + ELECTRODE3_TOUCH;
#endif

  DISABLE_TSI;

}

/********************************************************************************
 *   TSI_isr: TSI interrupt subroutine
 * Notes:
 *    -
 ********************************************************************************/

void TSI_isr(void)
{
  static uint16 TouchEvent = 0;
  uint16 lValidTouch = 0;
  uint16 l16Counter;

  TSI0_GENCS |= TSI_GENCS_OUTRGF_MASK;
  TSI0_GENCS |= TSI_GENCS_EOSF_MASK;


  /* Process electrode 0 */
  l16Counter = ELECTRODE0_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE0]) //See if detected a touch
  {
    TouchEvent |= (1<<ELECTRODE0); //Set touch event variable
    g32DebounceCounter[ELECTRODE0]--; //Decrement debounce counter
    if(!g32DebounceCounter[ELECTRODE0]) //If debounce counter reaches 0....
    {
      lValidTouch |= ((1<<ELECTRODE0)); //Signal that a valid touch has been detected
      TouchEvent &= ~(1<<ELECTRODE0);  //Clear touch event variable
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE0); //Clear touch event variable
    g32DebounceCounter[ELECTRODE0] = DBOUNCE_COUNTS; //Reset debounce counter
  }
  /***********************/

  /* Process electrode 1 */
  l16Counter = ELECTRODE1_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE1])
  {
    TouchEvent |= (1<<ELECTRODE1);
    g32DebounceCounter[ELECTRODE1]--;
    if(!g32DebounceCounter[ELECTRODE1])
    {
      lValidTouch |= ((1<<ELECTRODE1));
      TouchEvent &= ~(1<<ELECTRODE1);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE1);
    g32DebounceCounter[ELECTRODE1] = DBOUNCE_COUNTS;
  }
  /***********************/

#if ((defined(TWR_K60N512)) | (defined(TWR_K40X256)))  

  /* Process electrode 2 */
  l16Counter = ELECTRODE2_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE2])
  {
    TouchEvent |= (1<<ELECTRODE2);
    g32DebounceCounter[ELECTRODE2]--;
    if(!g32DebounceCounter[ELECTRODE2])
    {
      lValidTouch |= ((1<<ELECTRODE2));
      TouchEvent &= ~(1<<ELECTRODE2);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE2);
    g32DebounceCounter[ELECTRODE2] = DBOUNCE_COUNTS;
  }
  /***********************/

  /* Process electrode 3 */
  l16Counter = ELECTRODE3_COUNT;
  if(l16Counter>g16ElectrodeTouch[ELECTRODE3])
  {
    TouchEvent |= (1<<ELECTRODE3);
    g32DebounceCounter[ELECTRODE3]--;
    if(!g32DebounceCounter[ELECTRODE3])
    {
      lValidTouch |= ((1<<ELECTRODE3));
      TouchEvent &= ~(1<<ELECTRODE3);
    }
  }
  else
  {
    TouchEvent &= ~(1<<ELECTRODE3);
    g32DebounceCounter[ELECTRODE3] = DBOUNCE_COUNTS;
  }
  
#endif
  /***********************/

  if(lValidTouch&((1<<ELECTRODE0))) //If detected a valid touch...
  {
    LED0_TOGGLE; //Toggle LED
    lValidTouch &= ~((1<<ELECTRODE0)); //Clear valid touch
  }
  if(lValidTouch&((1<<ELECTRODE1)))
  {
    LED1_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE1));
  }
  
#if ((defined(TWR_K60N512)) | (defined(TWR_K40X256)))  
  
  if(lValidTouch&((1<<ELECTRODE2)))
  {
    LED2_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE2));
  }
  if(lValidTouch&((1<<ELECTRODE3)))
  {
    LED3_TOGGLE;
    lValidTouch &= ~((1<<ELECTRODE3));
  }
#endif

  TSI0_STATUS = 0xFFFFFFFF;
}

