/*
 * File:		hscmp.c
 * Purpose:		HSCMP example on HSCMP0
 *
 *                      P input is DAC output (1/2 VREF for this example, or 1.65V)
 *                      M input is IR receiver signal on PTC6
 *
 *                      This example uses IR transmitter on PTD7 to drive the
 *                         IR receiver. They are angled so that the IR transmitter
 *                         can drive the IR receiver on the tower board. This
 *                         requires shunting J24.
 *
 *                      Alternatively you can point a remote to the IR
 *                        receiver and toggle the HSCMP that way.
 *
 */


#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))

#include "common.h"

void hscmp_clear();
/********************************************************************/
void main (void)
{
  //Use HSCMP0 for this example
  CMP_MemMapPtr cmpch = CMP0_BASE_PTR;;

  printf("HSCMP0 Example\n");
  printf("Please shunt J24 in order to drive the IR Transmitter.\n");  
  printf("Press any key to continue\n");
  in_char(); //wait for keyboard input
  printf("\n\n");

  //Enable HSCMP module clock
  SIM_SCGC4 |=SIM_SCGC4_CMP_MASK;

  //Enable PTD7 as GPIO output to drive IR transmitter on tower board
  PORTD_PCR7=(0|PORT_PCR_MUX(1));
  GPIOD_PDDR=GPIO_PDDR_PDD(GPIO_PIN(7));	

  //Enable HSCMP0 Compare Out signal on PTC5
  PORTC_PCR5=PORT_PCR_MUX(6);  //CMP0_OUT on PTC5

  printf("CMP0_IN0 input is  on PTC6, which is connected to the IR receiver on the tower board\n");
  printf("CMP0_OUT output is on PTC5\n");
  printf("\tTWR-K40X256: This is on B33 on the TWR-ELEV\n");
  printf("\tTWR-K60N612: This is on A70 on the TWR-ELEV\n");

  //Set up HSCMP0 in NVIC. ISR put into vector table in isr.h file
  enable_irq(59);   //CMP0 Vector is 75. IRQ# is 75-16=59

  // Configure CMP.
  hscmp_clear(); //clear comparator registers
  CMP_CR0_REG(cmpch) = 0x00;  // Filter and digital hysteresis disabled
  CMP_CR1_REG(cmpch) = 0x15;  // Continuous mode, high-speed compare, unfiltered output, output pin disabled
  CMP_FPR_REG(cmpch) = 0x00;  // Filter disabled
  CMP_SCR_REG(cmpch) = 0x1E;  // Enable rising and falling edge interrupts, clear flags
  CMP_DACCR_REG(cmpch) = 0xE0; // 6-bit reference DAC enabled, output set to 1/2-scale, Vdd reference
  CMP_MUXCR_REG(cmpch) = 0xC7; // P-input as external channel 0. M-input as 6b DAC
  CMP_CR1_REG(cmpch) |= CMP_CR1_OPE_MASK; // Enable output pin for HSCMP0.

  while(1)
  {
    //Toggle PTD7 for IR transmitter to drive CMP0_IN0 connected to IR Receiver
    GPIOD_PTOR|=GPIO_PDOR_PDO(GPIO_PIN(7));

    //Delay for next toggle
    time_delay_ms(500);
  }

}
/********************************************************************/

/*
 * ISR for cmp0
 */
void cmp0_isr(void)
{
  //If rising edge
  if ((CMP0_SCR & CMP_SCR_CFR_MASK)==CMP_SCR_CFR_MASK)
  {
    CMP0_SCR |= CMP_SCR_CFR_MASK;  // Clear the flag
    printf("Rising edge on HSCMP0\n");
  }

  //If falling edge
  if ((CMP0_SCR & CMP_SCR_CFF_MASK)==CMP_SCR_CFF_MASK)
  {
    CMP0_SCR |= CMP_SCR_CFF_MASK;  // Clear the flag
    printf("Falling edge on HSCMP0\n");
  }
}


/*
 * Clear HSCMP0 registers
 */
void hscmp_clear()
{
  CMP_MemMapPtr cmpch = CMP0_BASE_PTR;

  CMP_CR0_REG(cmpch) = 0;
  CMP_CR1_REG(cmpch) = 0;
  CMP_FPR_REG(cmpch) = 0;
  CMP_SCR_REG(cmpch) = 0x06;  // Clear flags if set.
  CMP_DACCR_REG(cmpch) = 0;
  CMP_MUXCR_REG(cmpch) = 0;
}
