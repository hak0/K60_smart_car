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
#undef  VECTOR_036
#define VECTOR_036 pmc_lvd_isr
#undef  VECTOR_037
#define VECTOR_037 llwu_isr
#undef  VECTOR_080
#define VECTOR_080 ftm2_isr
#undef  VECTOR_082
#define VECTOR_082 rtc_isr
#undef  VECTOR_100
#define VECTOR_100 mcg_isr
#undef  VECTOR_101
#define VECTOR_101 lpt_isr
#undef  VECTOR_103
#define VECTOR_103 porta_isr
#undef  VECTOR_104
#define VECTOR_104 portb_isr
#undef  VECTOR_105
#define VECTOR_105 portc_isr
#undef  VECTOR_106
#define VECTOR_106 portd_isr
#undef  VECTOR_107
#define VECTOR_107 porte_isr

 void default_isr(void);
 void nmi_isr(void);
 void llwu_isr(void);
 void mcg_isr(void);
 void porta_isr(void);
 void portb_isr(void);
 void portc_isr(void);
 void portd_isr(void);
 void porte_isr(void);
 void pmc_lvd_isr(void);
 void lpt_lp_isr(void);
 void rtc_isr(void);
 void lpt_isr(void);
 void ftm2_isr(void); 

#endif  //__ISR_H

/* End of "isr.h" */
