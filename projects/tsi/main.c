/*
 * File:		main.c
 * Purpose:		TSI Example
 *
 *                      Toggle the LED's on the tower board by pressing the TSI
 *                        touch pads
 *
 */

#include "common.h"
#include "TSI.h"
#include "misc.h"

void GPIO_Init(void);

/********************************************************************/
void main (void)
{
  TSI_Init();
  GPIO_Init();
  TSI_SelfCalibration();

  START_SCANNING;
  ENABLE_EOS_INT;
  ENABLE_TSI;

  while(1)
  {

  }
}
/********************************************************************/

