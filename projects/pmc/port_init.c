
#include "common.h"
void Port_Init(void)
{
// setup ports and clock gating

  // Enable clock gating
//  SIM_SCGC1 = 0x0;
  SIM_SCGC2 = 0x0;
  SIM_SCGC3 = SIM_SCGC3_FTM2_MASK | SIM_SCGC3_ADC1_MASK;
  SIM_SCGC4 = SIM_SCGC4_UART0_MASK |SIM_SCGC4_UART1_MASK |SIM_SCGC4_UART2_MASK |SIM_SCGC4_UART3_MASK;
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK
               | SIM_SCGC5_PORTD_MASK |SIM_SCGC5_PORTE_MASK | SIM_SCGC5_LPTIMER_MASK;
  SIM_SCGC6 = SIM_SCGC6_FTFL_MASK;
  SIM_SCGC7 = SIM_SCGC7_FLEXBUS_MASK;
   
// select PTA6 to be FB_CLK to display bus clock
    PORTA_PCR6  = PORT_PCR_MUX(5) | PORT_PCR_DSE_MASK; // Set PORTA bit 6 as Alt 5 FB_CLKOUT
#if (defined(TWR_K60N512))
  PORTA_PCR10 = PORT_PCR_MUX(3);  // Port A10 as FTM2_CH0
#endif
  
#if (defined(TWR_K40X256))
    PORTC_PCR7 = PORT_PCR_MUX(1);  // Port C7 as GPIO
    GPIOC_PDDR = 0x80;       // Set PORTC bit 7 as output
#endif
}
