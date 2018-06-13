/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1


/* Example */
/*
#undef  VECTOR_101
#define VECTOR_101 lpt_isr


// ISR(s) are defined in your project directory.
extern void lpt_isr(void);
*/

#undef  VECTOR_097 // the vector number for your module, found in vector.h
#define VECTOR_097 dac0_12bit_isr


#undef  VECTOR_098 // the vector number for your module, found in vector.h
#define VECTOR_098 dac1_12bit_isr

extern void dac0_12bit_isr(void);
extern void dac1_12bit_isr(void);


#endif  //__ISR_H

/* End of "isr.h" */
