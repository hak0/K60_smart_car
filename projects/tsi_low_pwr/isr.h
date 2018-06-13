/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1


/* Example */

#undef  VECTOR_099
#define VECTOR_099 TSI_isr


// ISR(s) are defined in your project directory.
extern void TSI_isr(void);


#endif  //__ISR_H

/* End of "isr.h" */
