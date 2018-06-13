/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1


/* Example */

#undef  VECTOR_103
#define VECTOR_103 porta_isr

#undef  VECTOR_105
#define VECTOR_105 portc_isr

#undef  VECTOR_107
#define VECTOR_107 porte_isr

// ISR(s) are defined in your project directory.
extern void porta_isr(void);
extern void portc_isr(void);
extern void porte_isr(void);


#endif  //__ISR_H

/* End of "isr.h" */
