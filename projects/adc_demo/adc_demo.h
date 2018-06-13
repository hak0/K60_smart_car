/******************************************************************************
* File:    ADC_PDB.h
* Purpose: header file for ADC_PDB.c
* Note: 
******************************************************************************/

#ifndef __ADC_PDB_H
#define __ADC_PDB_H 1



/*     #defines  */

#define PIN_LOW     GPIOA_PCOR = 0x01 << 29;               // clear
#define PIN_HIGH    GPIOA_PSOR = 0x01 << 29;               // set


void delay(void);
void Init_Gpio(void);


// This is included in case some filtering is needed on the ADC input
// to faciliate DSP code developments

#include "dsp.h"

#endif  //__ADC_PDB_H

/* End of "ADC_PDB.h" */
