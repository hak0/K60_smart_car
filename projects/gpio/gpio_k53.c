/*
 * File:		gpio_k53.c
 * Purpose:		LED and Switch Example
 *
 *                      Configures GPIO for the LED and push buttons on the TWR-K53N512
 *                      Orange LED - On if SW1 pressed
 *                      Yellow LED - Toggles on/off
 *
 *                      Also configures push button SW1 for falling IRQ's. ISR
 *                      configured in vector table in isr.h
 *
 */


#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))

#include "common.h"

//Function declarations
void porta_isr(void);
void portc_isr(void);
void porte_isr(void);
void init_gpio(void);
void delay(void);

/********************************************************************/

void main (void)
{
  printf("TWR-K53N512 GPIO Example!\n");

  /* Turn on all port clocks */
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

  /* Enable GPIOC interrupts in NVIC */
  enable_irq(89); //GPIOC Vector is 105. IRQ# is 105-16=89

  /* Initialize GPIO on TWR-K53N512 */
  init_gpio();

  while(1)
  {
    //Toggle the yellow LED on PTC8
    GPIOC_PTOR|=GPIO_PDOR_PDO(GPIO_PIN(8));

    //Look at status of SW1 on PTC5
    if((GPIOC_PDIR & GPIO_PDIR_PDI(GPIO_PIN(5)))==0) //if pressed...
    {
      GPIOC_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(7)); //Set PTC7 to 0 (turns on orange LED)
    }
    else //else if SW1 not pressed...
    {
      GPIOC_PDOR|=GPIO_PDOR_PDO(GPIO_PIN(7)); //Set PTC7 to 1 (turns off orange LED)
    }

    delay();
  }
}

/*
 * Initialize GPIO for Tower switches and LED's
 *
 *   PTC5 - SW1
 *
 *   PTC8 - Yellow (D7)
 *   PTC7 - Orange (D6)
 *
 */
void init_gpio()
{
  //Set PTC5 (connected to SW1) for GPIO functionality, falling IRQ,
  //   and to use internal pull-ups. (pin defaults to input state)
  PORTC_PCR5=PORT_PCR_MUX(1)|PORT_PCR_IRQC(0xA)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;

  //Set PTC7 and PTC8 (connected to LED's) for GPIO functionality
  PORTC_PCR7=(0|PORT_PCR_MUX(1));
  PORTC_PCR8=(0|PORT_PCR_MUX(1));

  //Change PTC7 and PTC8 outputs
  GPIOC_PDDR=GPIO_PDDR_PDD(GPIO_PIN(7) | GPIO_PIN(8) );	
}

/********************************************************************/

/*
 * ISR for PORTA interrupts
 */
void porta_isr(void)
{
  PORTA_ISFR=0xFFFFFFFF;  //Clear Port A ISR flags
}

/*
 * ISR for PORTE interrupts
 */
void porte_isr(void)
{
  PORTE_ISFR=0xFFFFFFFF;  //Clear Port E ISR flags
}

/*
 * ISR for PORTC interrupts
 */
void portc_isr(void)
{
  PORTC_ISFR=0xFFFFFFFF;  //Clear Port C ISR flags
  printf("SW1 Pressed\n");
}


/*
 * Function to provide a short delay
 */
void delay()
{
  unsigned int i, n;
  for(i=0;i<30000;i++)
  {
    for(n=0;n<200;n++)
    {
      asm("nop");
    }
  }
}