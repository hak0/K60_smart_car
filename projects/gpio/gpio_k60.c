/*
 * File:		gpio_k60.c
 * Purpose:		LED and Switch Example
 *
 *                      Configures GPIO for the LED and push buttons on the TWR-K60N512
 *                      Blue LED - On
 *                      Green LED - Toggles on/off
 *                      Orange LED - On if SW2 pressed
 *                      Yellow LED - On if SW1 pressed
 *
 *                      Also configures push buttons for falling IRQ's. ISR
 *                        configured in vector table in isr.h
 *
 */


#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))

#include "common.h"

//Function declarations
void porta_isr(void);
void porte_isr(void);
void init_gpio(void);
void delay(void);

/********************************************************************/

void main (void)
{
  printf("TWR-K60N512 GPIO Example!\n");

  /* Turn on all port clocks */
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

  /* Enable GPIOA and GPIOE interrupts in NVIC */
  enable_irq(87); //GPIOA Vector is 103. IRQ# is 103-16=87
  enable_irq(91); //GPIOE Vector is 107. IRQ# is 107-16=91

  /* Initialize GPIO on TWR-K60N512 */
  init_gpio();

  while(1)
  {
    //Set PTA10 to 0 (turns on blue LED)
    GPIOA_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(10));

    //Toggle the green LED on PTA29
    GPIOA_PTOR|=GPIO_PDOR_PDO(GPIO_PIN(29));

    //Look at status of SW1 on PTA19
    if((GPIOA_PDIR & GPIO_PDIR_PDI(GPIO_PIN(19)))==0) //if pressed...
    {
      GPIOA_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(28)); //Set PTA28 to 0 (turns on yellow LED)
    }
    else //else if SW1 not pressed...
    {
      GPIOA_PDOR|=GPIO_PDOR_PDO(GPIO_PIN(28)); //Set PTA28 to 1 (turns off yellow LED)
    }

    //Look at status of SW2 on PTE26
    if((GPIOE_PDIR & GPIO_PDIR_PDI(GPIO_PIN(26)))==0)  //If pressed...
    {
      GPIOA_PDOR&=~GPIO_PIN(11); //Set PTA11 to 0 (turns on orange LED)
    }
    else //else if SW2 not pressed...
    {
      GPIOA_PDOR|=GPIO_PIN(11); //Set PTA11 to 1 ((turns off orange LED)
    }

    delay();
  }
}

/*
 * Initialize GPIO for Tower switches and LED's
 *
 *   PTA19 - SW1
 *   PTE26 - SW2
 *
 *   PTA10 - Blue (E4)
 *   PTA29 - Green (E3)
 *   PTA28 - Yellow (E2)
 *   PTA11 - Orange (E1)
 *
 */
void init_gpio()
{
  //Set PTA19 and PTE26 (connected to SW1 and SW2) for GPIO functionality, falling IRQ,
  //   and to use internal pull-ups. (pin defaults to input state)
  PORTA_PCR19=PORT_PCR_MUX(1)|PORT_PCR_IRQC(0xA)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;
  PORTE_PCR26=PORT_PCR_MUX(1)|PORT_PCR_IRQC(0xA)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK;

  //Set PTA10, PTA11, PTA28, and PTA29 (connected to LED's) for GPIO functionality
  PORTA_PCR10=(0|PORT_PCR_MUX(1));
  PORTA_PCR11=(0|PORT_PCR_MUX(1));
  PORTA_PCR28=(0|PORT_PCR_MUX(1));
  PORTA_PCR29=(0|PORT_PCR_MUX(1));

  //Change PTA10, PTA11, PTA28, PTA29 to outputs
  GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );	
}

/********************************************************************/

/*
 * ISR for PORTA interrupts
 */
void porta_isr(void)
{
  PORTA_ISFR=0xFFFFFFFF;  //Clear Port A ISR flags
  printf("SW1 Pressed\n");
}

/*
 * ISR for PORTE interrupts
 */
void porte_isr(void)
{
  PORTE_ISFR=0xFFFFFFFF;  //Clear Port E ISR flags
  printf("SW2 Pressed\n");
}

/*
 * ISR for PORTC interrupts
 */
void portc_isr(void)
{
  PORTC_ISFR=0xFFFFFFFF;  //Clear Port C ISR flags
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