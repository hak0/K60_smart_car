/*
 * File:        dac12bit.h
 * Purpose:     Function prototypes 
 *
 * Notes:
 */
#include "common.h"

#ifndef _DAC12BIT_H 
#define _DAC12BIT_H 1


#define _DAC0_H
#define _DAC1_H
 
 

/* Types definition */
typedef unsigned char byte;
typedef unsigned int word;
typedef unsigned long dword;
typedef unsigned long dlong[2];

 

/* DAXx registers reset values*/
#define DACx_DAT_RESET 0
#define DACx_SR_RESET 2
#define DACx_C0_RESET 0
#define DACx_C1_RESET 0
#define DACx_C2_RESET 15 //0x0f

/* DACx_C0 bits definition*/ 

#define DAC_DISABLE   0x00
#define DAC_ENABLE    DAC_C0_DACEN_MASK

#define DAC_SEL_VREFO  0x00
#define DAC_SEL_VDDA   DAC_C0_DACRFS_MASK 

#define DAC_SEL_PDB_HW_TRIG  0x00
#define DAC_SEL_SW_TRIG  DAC_C0_DACTRGSEL_MASK 

#define DAC_SW_TRIG_STOP 0x00
#define DAC_SW_TRIG_NEXT  DAC_C0_DACSWTRG_MASK 

#define DAC_HP_MODE  0x00
#define DAC_LP_MODE  DAC_C0_LPEN_MASK 

#define DAC_BFWM_INT_DISABLE  0x00
#define DAC_BFWM_INT_ENABLE   DAC_C0_DACBWIEN_MASK

#define DAC_BFT_PTR_INT_DISABLE 0x00
#define DAC_BFT_PTR_INT_ENABLE DAC_C0_DACBTIEN_MASK

#define DAC_BFB_PTR_INT_DISABLE 0x00
#define DAC_BFB_PTR_INT_ENABLE DAC_C0_DACBBIEN_MASK

/* DACx_C1 bits definition*/ 
#define DAC_DMA_DISABLE  0x00
#define DAC_DMA_ENABLE DAC_C1_DMAEN_MASK 

#define DAC_BFWM_1WORD DAC_C1_DACBFWM(0)  
#define DAC_BFWM_2WORDS DAC_C1_DACBFWM(1) 
#define DAC_BFWM_3WORDS DAC_C1_DACBFWM(2)  
#define DAC_BFWM_4WORDS DAC_C1_DACBFWM(3) 

#define DAC_BF_NORMAL_MODE DAC_C1_DACBFMD(0)
#define DAC_BF_SWING_MODE DAC_C1_DACBFMD(1) 
#define DAC_BF_ONE_TIME_MODE DAC_C1_DACBFMD(2)

#define DAC_BF_DISABLE 0x00
#define DAC_BF_ENABLE DAC_C1_DACBFEN_MASK 

/* DACx_C2 bits definition*/ 
#define DAC_SET_PTR_AT_BF(x) DAC_C2_DACBFRP(x)
#define DAC_SET_PTR_UP_LIMIT(x) DAC_C2_DACBFUP(x)
#endif /* _DAC12BIT_H */

 

#define Clear_DACBFWMF  0x03
#define Clear_DACBFRPTF 0x05
#define Clear_DACBFRPBF 0x06


void dac0_12bit_isr (void);
void dac1_12bit_isr (void);



void errorhandler (void);

void dac0_clk_enable (void);
void dac1_clk_enable (void);
void dac0_1_clk_enable(void);


void DAC12_VreferenceInit(DAC_MemMapPtr dacx_base_ptr,unsigned char Vinselect);
void DAC12_VreferenceRamp(void);
void DAC12_buffered (DAC_MemMapPtr dacx_base_ptr,byte WatermarkMode, byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos,byte BuffUpLimit);
void DAC12_SoftTrigBuffInit(DAC_MemMapPtr dacx_base_ptr,byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit);
void DAC12_Buff_Init_Plus256( DAC_MemMapPtr dacx_base_ptr);
int SET_DACx_BUFFER( DAC_MemMapPtr dacx_base_ptr, byte dacindex, int buffval);

void DAC12_Interrupt_Init(DAC_MemMapPtr dacx_base_ptr,byte watermark, byte zerobuffer, byte uplimitbuffer );
void DAC12_HWTrigBuff(DAC_MemMapPtr dacx_base_ptr, byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit);
void PDB_DAC0_TriggerInit(void);
void PDB_DAC1_TriggerInit(void);
void DACx_register_reset_por_values (DAC_MemMapPtr dacx_base_ptr);
void DACx_reset_dac0_1_reg_values(void);

void DAC12_WatermarkBuffInterrupt(DAC_MemMapPtr dacx_base_ptr, byte WatermarkBuffMode ,byte BuffMode, byte Vreference, byte TrigMode, byte BuffInitPos, byte BuffUpLimit);
void VREF_Init(void);
void DAC12_SoftwareTriggerLoop(void);


extern byte watermarkIndicator[16][16]; 
extern  int DACC2_DACBFUP ; 
extern  int DACC2_DACBFRP ; 
extern  int watermark ; 

 
    
extern    int bottom_isr;
extern    int top_isr;
extern    int watermark_isr ;
