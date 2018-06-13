/******************************************************************************
* File:    isr.h for the ADC_PDB project
* Purpose: Define interrupt service routines referenced by the vector table.
* Note: Only "vectors.c" should include this header file.
******************************************************************************/

#ifndef __ISR_H
#define __ISR_H 1
extern void adc0_isr(void);                                // ISR for ADC0
extern void adc1_isr(void);                                // ISR for ADC1
extern void pdb_isr(void);                                 // ISR for PDB

#undef  VECTOR_073   // as it was previously defined in vectors.h
#undef  VECTOR_074   // as it was previously defined in vectors.h
#undef  VECTOR_088   // as it was previously defined in vectors.h

                                     // address     vector irq    perihperal
#define VECTOR_073      adc0_isr     // 0x0000_0124 73     57     ADC0
#define VECTOR_074      adc1_isr     // 0x0000_0128 74     58     ADC1
#define VECTOR_088      pdb_isr      // 0x0000_0160 88     72     PDB
#endif  //__ISR_H

/* End of "isr.h" */

