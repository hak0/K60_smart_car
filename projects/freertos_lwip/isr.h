/******************************************************************************
* File:    isr.h
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1

#undef  VECTOR_011
#define VECTOR_011 vPortSVCHandler             // SVCall Handler

#undef  VECTOR_014
#define VECTOR_014 xPortPendSVHandler          // PendSV Handler

#undef  VECTOR_015
#define VECTOR_015 xPortSysTickHandler         // SysTick Handler


#undef  VECTOR_092
#define VECTOR_092 vENETISRHandler
  
#undef  VECTOR_093
#define VECTOR_093 vENETISRHandler
  
#undef  VECTOR_094
#define VECTOR_094 vENETISRHandler


// ISR(s) are defined in your project directory.
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

extern void vENETISRHandler( void );

#endif  //__ISR_H

/* End of "isr.h" */
