           
#include "common.h"
#include "peripheral.h"
#include "isr.h"
#include "mcg.h"

extern int core_clk_khz;
extern int core_clk_mhz;
extern int periph_clk_khz;

void Int_Vector_Set(void){
    extern uint32 __VECTOR_RAM[];  // Get vector table that was copied to RAM

      __VECTOR_RAM[36] = (uint32)pmc_lvd_isr; // Replace ISR
      NVICICPR0|=(1<<20);   //Clear any pending interrupts on LVD
      NVICISER0|=(1<<20);   //Enable interrupts from LVD module

      // Enable LLWU Interru[t in NVIC
      __VECTOR_RAM[37] = (uint32)llwu_isr; // Replace ISR
      NVICICPR0|=(1<<21);   //Clear any pending interrupts on LLWU
      NVICISER0|=(1<<21);   //Enable interrupts from LLWU module

      __VECTOR_RAM[82] = (uint32)rtc_isr; // Replace ISR
      NVICICPR2|=(1<<2);   //Clear any pending interrupts on RTC
      NVICISER2|=(1<<2);   //Enable interrupts from RTC module

       __VECTOR_RAM[101] = (uint32)lpt_isr; // Replace ISR
      NVICICPR2|=(1<<21);   //Clear any pending interrupts on LPT Timer
      NVICISER2|=(1<<21);   //Enable interrupts from LPT module

      __VECTOR_RAM[103] = (uint32)porta_isr; // Replace ISR
      NVICICPR2|=(1<<23);   //Clear any pending interrupts on Port A interrupt
      NVICISER2|=(1<<23);   //Enable interrupts from Port A

      // Enable Port B Interrupt in NVIC (vector 104)
      __VECTOR_RAM[104] = (uint32)portb_isr; // Replace ISR
      NVICICPR2|=(1<<24);   //Clear any pending interrupts on Port B
      NVICISER2|=(1<<24);   //Enable interrupts from Port B  

      __VECTOR_RAM[105] = (uint32)portc_isr; // Replace ISR
      NVICICPR2|=(1<<25);   //Clear any pending interrupts on Port C interrut
      NVICISER2|=(1<<25);   //Enable interrupts from Port C 

      __VECTOR_RAM[106] = (uint32)portd_isr; // Replace ISR
      NVICICPR2|=(1<<26);   //Clear any pending interrupts on Port D inteerut
      NVICISER2|=(1<<26);   //Enable interrupts from port D 
   
      __VECTOR_RAM[107] = (uint32)porte_isr; // Replace ISR
      NVICICPR2|=(1<<27);   //Clear any pending interrupts on Port E inteerut
      NVICISER2|=(1<<27);   //Enable interrupts from Port E 
 
      NVICICPR2|=(1);   //Clear any pending interrupts 
      NVICISER2|=(1);   //Enable interrupts from FTM2 module    

}
/********************************************************************/
/******************************************************************************
Function Name	:	Wait1ms
Engineer		:	
Date			:	20MAR07

Parameters		:	none
Returns			:	none
Notes			:	Waits 1mS. 
******************************************************************************/

void Wait1ms(void){
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;   // Clear the PIT time out flag
    while (PIT_TFLG0 & PIT_TFLG_TIF_MASK ==0){ }   // use PIT to wait 
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;   // Clear the PIT time out flag 
}
/******************************************************************************
Function Name	:	WaitNms
Engineer		:	
Date			:	20MAR07

Parameters		:	int
Returns			:	none
Notes			:	Waits for N mS. 
******************************************************************************/

void WaitNms(int n){
   int i;
   for(i=1;i<=n;i++) Wait1ms();
}

void LPT_Init(int count){
  /* Configure LPT */
  LPTMR0_CMR = LPTMR_CMR_COMPARE(count);  //Set compare value
  LPTMR0_PSR = LPTMR_PSR_PCS(0x1)|LPTMR_PSR_PBYP_MASK;  //Use internal 1khz clock
  LPTMR0_CSR = LPTMR_CSR_TIE_MASK;  //Enable LPT interrupt

}
/*
 * Zero out all registers.
 *
 */
void clear_lpt_registers(void)
{
  LPTMR0_CSR=0x00;
  LPTMR0_PSR=0x00;
  LPTMR0_CMR=0x00;
}


void sim_clk_gates_set(void){
  
    // Enable clock gating
  SIM_SCGC2 = 0x0;
  SIM_SCGC3 = SIM_SCGC3_FTM2_MASK | 
              SIM_SCGC3_ADC1_MASK;
  SIM_SCGC4 = SIM_SCGC4_UART0_MASK |
              SIM_SCGC4_UART1_MASK |
              SIM_SCGC4_UART2_MASK |
              SIM_SCGC4_UART3_MASK |
              SIM_SCGC4_LLWU_MASK;
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | 
              SIM_SCGC5_PORTB_MASK | 
              SIM_SCGC5_PORTC_MASK | 
              SIM_SCGC5_PORTD_MASK |
              SIM_SCGC5_PORTE_MASK | 
              SIM_SCGC5_LPTIMER_MASK |
              SIM_SCGC5_REGFILE_MASK;
  SIM_SCGC6 = SIM_SCGC6_FTFL_MASK |
              SIM_SCGC6_RTC_MASK;
  SIM_SCGC7 = SIM_SCGC7_FLEXBUS_MASK;
    
}
void Port_Init(void)
{
// setup ports


// select PTA6 to be FB_CLK to display bus clock
    PORTA_PCR6  = PORT_PCR_MUX(5) | PORT_PCR_DSE_MASK; // Set PORTA bit 6 as Alt 5 FB_CLKOUT
#if (defined(TWR_K60N512))
  PORTA_PCR10 = PORT_PCR_MUX(3);  // Port A10 as FTM2_CH0
  PORTE_PCR26 = (PORT_PCR_ISF_MASK |    //  clear Flag if there
              PORT_PCR_MUX(01) |     //  GPIO
              PORT_PCR_IRQC(0x0A) |  //  falling edge enable
              PORT_PCR_PE_MASK |     //  Pull enable
              PORT_PCR_PS_MASK);     //  pull up enable

  PORTA_PCR19 = (PORT_PCR_ISF_MASK |    //  clear Flag if there
              PORT_PCR_MUX(01) |     //  GPIO
              PORT_PCR_IRQC(0x09) |  //  rising edge enable
              PORT_PCR_PE_MASK |     //  Pull enable
              PORT_PCR_PS_MASK);     //  pull up enable

#endif
  
#if (defined(TWR_K40X256))
//  SW6 and SW7
  PORTC_PCR5  = (PORT_PCR_ISF_MASK |    //  clear Flag if there
              PORT_PCR_MUX(01) |     //  GPIO
              PORT_PCR_IRQC(0x0A) |  //  falling edge enable
              PORT_PCR_PE_MASK |     //  Pull enable
              PORT_PCR_PS_MASK);     //  pull up enable

  PORTC_PCR13  = (PORT_PCR_ISF_MASK |    //  clear Flag if there
              PORT_PCR_MUX(01) |     //  GPIO
              PORT_PCR_IRQC(0x0A) |  //  falling edge enable
              PORT_PCR_PE_MASK |     //  Pull enable
              PORT_PCR_PS_MASK);     //  pull up enable


  PORTC_PCR7 = PORT_PCR_MUX(1);  // Port C7 as GPIO LED D16
  GPIOC_PDDR |= 0x80;       // Set PORTC bit 7 as output
#endif
                                  
}
void LVD_Init(void)
{
/* setup LVD
Low-Voltage Detect Voltage Select
Selects the LVD trip point voltage (VLVD).
00 Low trip point selected (VLVD = VLVDL)
01 High trip point selected (VLVD = VLVDH)
10 Reserved
11 Reserved
  */
/* Choose one of the following statements */
PMC_LVDSC1 &= ~PMC_LVDSC1_LVDRE_MASK ;   //Disable LVD Reset
//PMC_LVDSC1 |= PMC_LVDSC1_LVDRE_MASK ;   //Enable LVD Reset

/* Choose one of the following statements */
//PMC_LVDSC1 |= PMC_LVDSC1_LVDV_MASK & 0x01;  //High Trip point 2.48V
PMC_LVDSC1 &= PMC_LVDSC1_LVDV_MASK & 0x00;  //Low Trip point 1.54 V
 
/* Choose one of the following statements */
PMC_LVDSC2  = PMC_LVDSC2_LVWACK_MASK | PMC_LVDSC2_LVWV(0); //0b00 low trip point LVWV
//PMC_LVDSC2  = PMC_LVDSC2_LVWACK_MASK | PMC_LVDSC2_LVWV(1); //0b01 mid1 trip point LVWV
//PMC_LVDSC2  = PMC_LVDSC2_LVWACK_MASK | PMC_LVDSC2_LVWV(2); //0b01000010 mid2 trip point LVWV
//PMC_LVDSC2  = PMC_LVDSC2_LVWACK_MASK | PMC_LVDSC2_LVWV(3); //0b01000011 high trip point LVWV

// ack to clear initial flags
PMC_LVDSC1 |= PMC_LVDSC1_LVDACK_MASK; // clear detect flag if present
PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK; // clear warning flag if present

/* 
LVWV if LVDV high range selected
   Low trip point selected (VLVW = VLVW1)   - 2.62
   Mid 1 trip point selected (VLVW = VLVW2) - 2.72
   Mid 2 trip point selected (VLVW = VLVW3) - 2.82
   High trip point selected (VLVW = VLV4)   - 2.92
LVWV if LVDV low range selected
   Low trip point selected (VLVW = VLVW1)   - 1.74
   Mid 1 trip point selected (VLVW = VLVW2) - 1.84
   Mid 2 trip point selected (VLVW = VLVW3) - 1.94
   High trip point selected (VLVW = VLV4)   - 2.04
*/
}
void LLWU_Init(void){
  printf("[LLWU_Init] LLWU initialize routine \n\n");

  //enable wakeup pin enable NMI - PTA4 - WUPE3
  LLWU_PE1 |= (LLWU_PE1_WUPE3(2)| LLWU_PE1_WUPE2(2)| LLWU_PE1_WUPE1(2) |LLWU_PE1_WUPE0(2));  
  LLWU_PE2 |= (LLWU_PE2_WUPE7(2)| LLWU_PE2_WUPE6(2)| LLWU_PE2_WUPE5(2) |LLWU_PE2_WUPE4(2));  
  LLWU_PE3 |= (LLWU_PE3_WUPE11(2)| LLWU_PE3_WUPE10(2)| LLWU_PE3_WUPE9(2) |LLWU_PE3_WUPE8(2));  
  LLWU_PE4 |= (LLWU_PE4_WUPE15(2)| LLWU_PE4_WUPE14(2)| LLWU_PE4_WUPE13(2) |LLWU_PE4_WUPE12(2));  
  printf("[LLWU_Init] modules 3,2,1,0     enabled;  LLWU PE1  = %#02X \r\n",    (LLWU_PE1)  );
  printf("[LLWU_Init] modules 7,6,5,4     enabled;  LLWU PE2  = %#02X \r\n",    (LLWU_PE2)  );
  printf("[LLWU_Init] modules 11,10,9,8   enabled;  LLWU PE3  = %#02X \r\n",    (LLWU_PE3)  );
  printf("[LLWU_Init] modules 15,14,13,12 enabled;  LLWU PE4  = %#02X \r\n",    (LLWU_PE4)  );
  //falling edge on all pins

  LLWU_CS = 0x07;  //enable reset filter
  printf("[LLWU_Init]enable reset filter enabled;  LLWU_CS  = %#02X \r\n",    (LLWU_CS)  );
  
  //LLWU_M5IF
  // enable all modules to wakeup from low power modes
  LLWU_ME |= (LLWU_ME_WUME7_MASK |LLWU_ME_WUME6_MASK |LLWU_ME_WUME5_MASK \
             |LLWU_ME_WUME4_MASK |LLWU_ME_WUME3_MASK |LLWU_ME_WUME2_MASK \
             |LLWU_ME_WUME1_MASK | LLWU_ME_WUME0_MASK);
  printf("[LLWU_Init]All modules enabled;  LLWU ME  = %#02X \r\n",    (LLWU_ME)  );

  NVICISER0 = NVIC_ISER_SETENA_MASK;  //enable llwu interrupts in the NVIC
     

}

void RTC_Init(void){
   // enable the external 32768 oscillator
   //clear the Time Seconds Register
   printf(" RTC initialization routine \n\n");
   RTC_SR |=  RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;   
   RTC_TSR = 0x00;
   // enable the oscillator and set load capacitors
   RTC_CR |=  RTC_CR_OSCE_MASK ;
   // Ennable the timer counter and clear the flags
   //set the Timer Alarm Register to 0 to disable RTC flag
   RTC_TAR = 0x01;  // set this register to other than 00 so clear TAF
   // LLRSTEN The LLRSTEN bit was not implemented for P2 rev1.  - It is mentioned in several places.
   printf("RTC_TSR  = %#02X \r\n",    (RTC_TSR ))  ;
   printf("RTC_TPR  = %#02X \r\n",    (RTC_TPR ))  ;
   printf("RTC_TAR  = %#02X \r\n\n",  (RTC_TAR) )  ;
   printf("RTC_TCR  = %#02X \r\n",    (RTC_TCR ))  ;
   printf("RTC_CR   = %#02X \r\n",    (RTC_CR ))  ;
   printf("RTC_SR   = %#02X \r\n\n",  (RTC_SR) )  ;
   printf("RTC_LR   = %#02X \r\n",    (RTC_LR ))  ;
   printf("RTC_IER  = %#02X \r\n",    (RTC_IER ))  ;
   printf("RTC_WAR  = %#02X \r\n\n",  (RTC_WAR) )  ;
   printf("RTC_RAR  = %#02X \r\n\n",  (RTC_RAR) )  ;
   
}


/********************************************************************************
 *   delay: delay
 * Notes:
 *    - 
 ********************************************************************************/
void delay(void)
{
  unsigned int i, n, j;
  for(i=0;i<3000;i++)  //delay
  {
    for(n=0;n<1000;n++)
    {
      j=j*j*j;
    }
  }
}


void ftm_enable(void)
{
      // Initialize FTM2 for output compare
    FTM2_MODE |= FTM_MODE_FTMEN_MASK;  // Enable FTM2, non-TPM-compatible registers.
    FTM2_CNT = 0x00000000;  // Clear counter.
    FTM2_MOD = FTM_MOD_MOD(0xFFFF);  // FTM0 Modulo set for counter overflow after 1 system clock cycles 
    FTM2_C0SC = FTM_CnSC_MSA_MASK | FTM_CnSC_ELSA_MASK; // FTM2_CH0 set for Output Compare mode, toggle output on match
    FTM2_C0V = FTM_CnV_VAL(0xFFFF);  // FTM2_CH0 output compare action on counter value of 0.
    FTM2_OUTINIT |= (FTM_OUTINIT_CH0OI_MASK);  // Static output level high for FTM2_CH0 when OC toggle not desired.
    FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(6);  // FTM0 in up-counter mode, TOF interrupt disabled, system clock source, div-by-128 prescaler.
#if (defined(TWR_K40X256))
    FTM2_SC |= FTM_SC_TOIE_MASK; // enable TOF interrupt if K40 tower board
#endif
}
