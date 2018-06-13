/*
 * File:		gpio_k40.c
 * Purpose:		LED and Switch Example
 *
 *                      Configures GPIO for the LED and push buttons on the TWR-K40X256
 *                      Blue LED - On
 *                      Green LED - Toggles on/off
 *                      Orange LED - On if SW4 pressed
 *                      Yellow LED - On if SW3 pressed
 *
 *                      Also configures push buttons for falling IRQ's. ISR
 *                        configured in vector table in isr.h
 */

#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))

#include "common.h"

//Function declarations
void init_gpio(void);
void delay(void);

/********************************************************************/

void main (void)
{
  printf("TWR-K40X256 GPIO Example!\n");

  /* Turn on all port clocks */
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

  /* Enable GPIOC interrupts in NVIC */
  enable_irq(89); //GPIOC Vector is 105. IRQ# is 105-16=89

  /* Initialize GPIO on TWR-K40X256 */
  init_gpio();

  while(1)
  {
    //Set PTB11 to 0 (turns on blue LED)
    GPIOB_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(11));

    //Toggle the green LED on PTC9
    GPIOC_PTOR|=GPIO_PDOR_PDO(GPIO_PIN(9));

    //Look at status of SW3 on PTC5
    if((GPIOC_PDIR & GPIO_PDIR_PDI(GPIO_PIN(5)))==0) //if pressed...
    {
      GPIOC_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(8)); //Set PTC8 to 0 (turns on yellow LED)
    }
    else  //else if not pressed...
    {
      GPIOC_PDOR|=GPIO_PDOR_PDO(GPIO_PIN(8)); //Set PTC8 to 1 (turns off yellow LED)
    }

    //Look at status of SW4 on PTC13
    if((GPIOC_PDIR & GPIO_PDIR_PDI(GPIO_PIN(13))) == 0)  //If pressed...
    {
      GPIOC_PDOR&=~GPIO_PDOR_PDO(GPIO_PIN(7)); //Set PTC7 to 0 (turns on orange LED)
    }
    else //else if not pressed...
    {
      GPIOC_PDOR|=GPIO_PDOR_PDO(GPIO_PIN(7)); //Set PTC7 to 1 (turns off orange LED)
    }

    delay();
  }
}

/*
 * Initialize GPIO for Tower switches and LED's
 *
 *   PTC5 - SW3
 *   PTC13 - SW4
 *
 *   PTB11 - Blue LED (E4)
 *   PTC9 - Green LED (E3)
 *   PTC8 - Yellow LED (E2)
 *   PTC7 - Orange LED (E1)
 *
 */
void init_gpio()
{
  //Set PTC5 and PTC13 (connected to SW3 and SW4) for GPIO functionality, falling IRQ,
  //   and to use internal pull-ups. (pin defaults to input state)
  PORTC_PCR5=(0|PORT_PCR_MUX(1)|PORT_PCR_IRQC(0xA)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK);
  PORTC_PCR13=(0|PORT_PCR_MUX(1)|PORT_PCR_IRQC(0xA)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK);

  //Set PTB11, PTC7, PTC8, and PTC9 (connected to LED's) for GPIO functionality
  PORTB_PCR11=(0|PORT_PCR_MUX(1));
  PORTC_PCR7=(0|PORT_PCR_MUX(1));
  PORTC_PCR8=(0|PORT_PCR_MUX(1));
  PORTC_PCR9=(0|PORT_PCR_MUX(1));

  //Change PTB11, PTC7, PTC8, PTC9 to outputs
  GPIOB_PDDR=GPIO_PDDR_PDD(GPIO_PIN(11));
  GPIOC_PDDR=GPIO_PDDR_PDD(GPIO_PIN(7) | GPIO_PIN(8) | GPIO_PIN(9));	
}
/********************************************************************/

/*
 * ISR for PORTC interrupts
 */
void portc_isr(void)
{
  if(PORTC_ISFR & PORT_ISFR_ISF(GPIO_PIN(5)))
  {
     printf("SW3 Pressed\n");
  }
  if(PORTC_ISFR & PORT_ISFR_ISF(GPIO_PIN(13)))
  {
     printf("SW4 Pressed\n");
  }
  PORTC_ISFR=0xFFFFFFFF;  //Clear Port C ISR flags
}

/*
 * ISR for PORTE interrupts
 */
void porte_isr(void)
{
  PORTE_ISFR=0xFFFFFFFF;  //Clear Port E ISR flags
}

/*
 * ISR for PORTA interrupts
 */
void porta_isr(void)
{
  PORTA_ISFR=0xFFFFFFFF;  //Clear Port A ISR flags
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