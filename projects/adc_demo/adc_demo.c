/*
 * File:		adc_demo.c
 * Purpose:		Demo the Analog to Digtal Converter triggered by the Programable Delay Block
 *                      Call the function Hw_Trig_Test which demonstrates and tests this function.
 *                      Runs on the Tower Kinetis Card, using the potentiometer as input.
 *                      The operator will be prompted on the serial port at 115200 baud, 8n1.
 *                      The LED's will also indicate the timing of the conversions.
 *
 */


#include "common.h"    // Common Kinetis defines
#include "adc16.h"     // ADC driver defines
#include "adc_demo.h"   // For this function

/*    global variable    */
tADC_Config Master_Adc_Config;  // This is the structure that contains the desired ADC/PGA configuration.


extern uint8_t  Hw_Trig_Test(void);

/********************************************************************/
void main (void)
{
 Init_Gpio(); 
 Hw_Trig_Test(); //  demo the adc/pdb
}  



//******************************************************************************
// setup an output pin, "PIN",  to indirectly observe adc status changes
// 
//******************************************************************************

void Init_Gpio(void)

{

 //the "PIN" will be visible as PORTA29 on TOWER board
 // setup PORTA29  for output "PIN" as called in test programs as on TOWER board
  PORTA_PCR29 = PORT_PCR_MUX(1) ;        // selec GPIO function
  GPIOA_PCOR = 0x01 << 29 ;              // initial out low 
  GPIOA_PDDR = 0x01 << 29 ;              // output enable 

}
