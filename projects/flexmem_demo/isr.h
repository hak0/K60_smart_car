/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1


/* Define PIT CH0 vector */
#undef  VECTOR_084
#define VECTOR_084 pit0_isr



// function prototype for pit ch0 isr
extern void pit0_isr(void);



#endif  //__ISR_H

/* End of "isr.h" */
