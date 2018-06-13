/******************************************************************************
* File:    peripheral.h
* Purpose: Define  routines referenced by main routines
* Note: 
******************************************************************************/

#ifndef __PERIPHERAL_H
#define __PERIPHERAL_H 1
           
#include "common.h"


/* function prototypes */
void Wait1ms(void);
void WaitNms(int n);
void out_srs(void);
void LPT_Init(int);
void clear_lpt_registers(void);
void sim_clk_gates_set(void);
void Port_Init(void);
void LVD_Init(void);
void RTC_Init(void);
void Int_Vector_Set(void);

void delay(void);
void ftm_enable(void);

#endif  //__PERIPHERAL_H


/* End of "peripheral.h" */
