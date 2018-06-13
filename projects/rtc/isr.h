/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1


#undef  VECTOR_082
#define VECTOR_082  rtc_isr
extern void rtc_isr(void);


#endif  //__ISR_H

/* End of "isr.h" */
