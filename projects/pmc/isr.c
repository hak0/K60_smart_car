/******************************************************************************
* File:    isr.c
*
* Purpose: Define custom interrupt service routines (ISRs).
******************************************************************************/
#include "vectors.h"
#include "isr.h"
#include "peripheral.h"
#include "common.h"
#include "uif.h"
#include "mcg.h"

extern unsigned char seconds_count;
extern uint32 rtc_num_sec_timr;
extern int redge_flag;    // Flag to indicate when Rising Edge Interrupt has occurred.
extern int fedge_flag;    // Flag to indicate when Falling Edge Interrupt has occurred.
extern int re_init_clk;
extern volatile unsigned char sec;
extern volatile unsigned char tsec;
extern volatile unsigned char mins;
extern volatile unsigned char tmins;
extern volatile unsigned char hour;
extern volatile unsigned char thour;
extern volatile unsigned char AMPM;  //0 = AM 1 = PM
extern volatile unsigned char clock_on;  //1 = Clock displayed on LCD 0 - not
 

/******************************************************************************
* lvd isr(void)
*
* LVD ISR definition.
*
* In:  n/a
* Out: n/a
******************************************************************************/

void pmc_lvd_isr(void){
    if(re_init_clk){
      mcg_pbe_2_pee();
    } 
    printf("\rPMC_LVD ISR entered** ");
    if ( PMC_LVDSC2 & PMC_LVDSC2_LVWF_MASK){
       GPIOC_PSOR = 0x00000001;       // set Port C0 indicate LVWF
       PMC_LVDSC2 |= PMC_LVDSC2_LVWACK_MASK;
       printf(" Warning Flag Set = %#02X  ",(PMC_LVDSC2 )) ;
    }else {
       GPIOC_PCOR = 0x00000001;       // clear Port C0 no LVWF
       printf(" Warning Flag Clr = %#02X  ",(PMC_LVDSC2 )) ;
    }  

    if ( PMC_LVDSC1 & PMC_LVDSC1_LVDF_MASK){
       GPIOC_PSOR = 0x00000002;       // set Port C1 indicate LVDF
       PMC_LVDSC1 |= PMC_LVDSC1_LVDACK_MASK;
       printf(" Detect Flag Set = %#02X ", (PMC_LVDSC1 )) ;
    }else {
       GPIOC_PCOR = 0x00000002;       // clear Port C1 no LVDF
       printf(" Detect Flag Clr = %#02X ", (PMC_LVDSC1 )) ;
    }
    printf("\r");
    NVICICPR0|=(1<<20);   //Clear any pending interrupts on LVD
    NVICISER0|=(1<<20);   //Enable interrupts from LVD module
}
/******************************************************************************
* llwu_isr(void)
*
* LLWU ISR definition.
*
* In:  n/a
* Out: n/a
******************************************************************************/

void llwu_isr(void)
{

   
   if ((MC_SRSL & MC_SRSL_WAKEUP_MASK) || ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 3)) {

      Int_Vector_Set();
      sim_clk_gates_set();
      Port_Init();
      LLWU_CS |= LLWU_CS_ACKISO_MASK; 
       
   }

   if(re_init_clk){
       mcg_pbe_2_pee();
   } 
   printf("\n****LLWU handler ISR entered*****\r\n");
   if (LLWU_F1 & LLWU_F1_WUF0_MASK) {
     printf("\n****WUF0 - PTE1/SCI1_RX/I2C1_SCL *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF0_MASK ;
     if (PORTE_PCR1 & PORT_PCR_ISF_MASK) {
        printf(" [LLWU ISR] Interrput flag on PORTE pin 1 set \n");
        PORTE_PCR1 |= PORT_PCR_ISF_MASK;     //  clear Flag if there
        //write 1 to ISF of port C pin 8 to clear interrupt flag
     }
   }
   if (LLWU_F1 & LLWU_F1_WUF1_MASK) {
     printf("\n [LLWU ISR] ****WUF1 - PTE2/DSPI1_SCK/SDHC0_DCLK *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF1_MASK ;
     if (PORTE_PCR2 & PORT_PCR_ISF_MASK) {
        printf(" [LLWU ISR] Interrput flag on PORTE pin 2 set \n");
        PORTE_PCR1 |= PORT_PCR_ISF_MASK;     //  clear Flag if there
        //write 1 to ISF of port C pin 8 to clear interrupt flag
     }
   }
   if (LLWU_F1 & LLWU_F1_WUF2_MASK) {
     printf("\n [LLWU ISR] ****WUF2 - PTE4/DSPI1_PCS0/SDHC0_D3 *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF2_MASK ;
     PORTE_PCR4 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F1 & LLWU_F1_WUF3_MASK){
     printf("\n [LLWU ISR] ****WUF3 - PTA4/FTM0_CH1/NMI *****\r\n");
     PORTA_PCR4 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port A pin 4 to clear interrupt flag
     LLWU_F1 |= LLWU_F1_WUF3_MASK ;
   }
   if (LLWU_F1 & LLWU_F1_WUF4_MASK) {
     printf("\n [LLWU ISR] ****WUF4 - PTA13/CAN0_RX/FTM1_CH1/FTM1_QD_PHB *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF4_MASK ;
     PORTA_PCR13 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F1 & LLWU_F1_WUF5_MASK) {
     printf("\n [LLWU ISR] ****WUF5 - PTB0/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF5_MASK ;
     PORTB_PCR0  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F1 & LLWU_F1_WUF6_MASK) {
     printf("\n [LLWU ISR] ****WUF6 - PTC1/SCI1_RTS/FTM0_CH0 *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF6_MASK ;
     PORTC_PCR1  |= PORT_PCR_ISF_MASK ;  //  clear Flag if there
   }
   if (LLWU_F1 & LLWU_F1_WUF7_MASK) {
     printf("\n [LLWU ISR] ****WUF7 - PTC3/SCI1_RX/FTM0_CH2 *****\r\n"); 
     LLWU_F1 |= LLWU_F1_WUF7_MASK ;
     PORTC_PCR3  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF8_MASK) {
     printf("\n [LLWU ISR] ****WUF8 - PTC4/DSPI0_PCS0/FTM0_CH3 *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF8_MASK ;
     PORTC_PCR4  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF9_MASK) {
     printf("\n [LLWU ISR] ****WUF9 - PTC5/DSPI0_SCK *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF9_MASK ;
     PORTC_PCR5 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF10_MASK) {
     printf("\n [LLWU ISR] ****WUF10 - PTC6/PDB0_EXTRG *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF10_MASK ;
     PORTC_PCR6  |= PORT_PCR_ISF_MASK;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF11_MASK) {
     printf("\n [LLWU ISR] ****WUF11 - PTC11/SSI0_RXD *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF11_MASK ;
     PORTC_PCR11 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF12_MASK) {
     printf("\n [LLWU ISR] ****WUF12 - PTD0/DSPI0_PCS0/SCI2_RTS *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF12_MASK ;
     PORTD_PCR0  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF13_MASK) {
     printf("\n [LLWU ISR] ****WUF13 - PTD2/SCI2_RX *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF13_MASK ;
     PORTD_PCR2  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there              //UART2 RX pin
   }
   if (LLWU_F2 & LLWU_F2_WUF14_MASK) {
     printf("\n [LLWU ISR] ****WUF14 - PTD4/SCI0_RTS/FTM0_CH4/EWM_IN *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF14_MASK ;
     PORTD_PCR4  |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
   }
   if (LLWU_F2 & LLWU_F2_WUF15_MASK) {
     printf("\n [LLWU ISR] ****WUF15 - PTD6/SCI0_RX/FTM0_CH6/FTM0_FLT0 *****\r\n"); 
     LLWU_F2 |= LLWU_F2_WUF15_MASK ;
     PORTD_PCR6  |= PORT_PCR_ISF_MASK  ;   //  clear Flag if there
   }
   if (LLWU_F3 & LLWU_F3_MWUF0_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF0 IF  LPT  *****\r\n"); 
     LPTMR0_CSR |=  LPTMR_CSR_TCF_MASK;   // write 1 to TCF to clear the LPT timer compare flag
 // used in LPT service routine.    LPTMR0_CSR = ( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK  );
   
   }
   if (LLWU_F3 & LLWU_F3_MWUF1_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF1 IF  CMP0 *****\r\n"); 
     CMP0_SCR |= CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK;   // write 1 to CCF to clear the CMP0 flag
   }
   if (LLWU_F3 & LLWU_F3_MWUF2_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF2 IF  CMP1 *****\r\n"); 
     CMP1_SCR |= CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK;   // write 1 to CCF and CFR to clear the CMP1 flag
   }
   if (LLWU_F3 & LLWU_F3_MWUF3_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF3 IF  CMP2 *****\r\n"); 
     CMP2_SCR |= CMP_SCR_CFF_MASK | CMP_SCR_CFR_MASK;   // write 1 to CCF and CFR to clear the CMP2 flag
   }
   if (LLWU_F3 & LLWU_F3_MWUF4_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF4 IF  TSI  - Need to clear the flag to exit *****\r\n"); 
     LLWU_F3 |= LLWU_F3_MWUF4_MASK;
   }
   if (LLWU_F3 & LLWU_F3_MWUF5_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF5 IF  RTC *****\r\n"); 
     RTC_SR |= RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;
     RTC_TAR = RTC_TSR-1 + seconds_count;   //wake up in x more sec
     NVICICPR2|=(1<<2);   //Clear any pending interrupts on RTC     
//     printf("WUF3_MWUF5 - RTC handled  = %#02X \r",    (RTC_TSR )) ;
   }
   if (LLWU_F3 & LLWU_F3_MWUF6_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF6 RESERVED  - WHY DID WE GET HERE???? *****\r\n"); 
   }
   if (LLWU_F3 & LLWU_F3_MWUF7_MASK) {
     printf("\n [LLWU ISR] ****WUF3_MWUF7 Error Detect - wake-up source unknown *****\r\n"); 
     LLWU_F3 |= LLWU_F3_MWUF7_MASK;
   }

   return;
}
void mcg_isr(void){
  if(re_init_clk){
      mcg_pbe_2_pee();
  } 
   printf("\n****MCG ISR entered*****\r\n\n");
  GPIOD_PDOR = 0x00000040; //set bit 6 of port D
  while (!(MCG_S & MCG_S_LOCK_MASK)){}; //wait for lock to set again
  GPIOD_PDOR = 0x00000000; //clear bit 6 of port D  
  MCG_S |= MCG_S_LOLS_MASK; // write 1 to LOLS to clear interrupt flag
}

void lpt_isr(void){
  if(re_init_clk){
       mcg_pbe_2_pee();
 } 
  printf("\n****LPT ISR entered*****\r\n");

  LPTMR0_CSR = ( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK  );
      // enable timer
      // enable interrupts
      // clear the flag

}
void porta_isr(void)
{
  if(re_init_clk){
      mcg_pbe_2_pee();
  } 
  if ( PORTA_PCR19 |= PORT_PCR_ISF_MASK){
     printf("\n****PORTA4 ISR entered*****\r\n\n");
     PORTA_PCR19 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port A pin 19 to clear interrupt flag
    printf("\n****PORTA19 ISR ISF cleared *****\r\n");
  } else  {
    printf("\n****PORTA19 ISR entered but ISF not set *****\r\n");
  } //end if else porta
}
void portb_isr(void)
{
  if(re_init_clk){
      mcg_pbe_2_pee();
  } 
  if ( PORTB_PCR4 |= PORT_PCR_ISF_MASK){
     printf("\n****PORTB4 ISR entered*****\r\n");
     PORTB_PCR4 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port B pin 0 to clear interrupt flag
     PORTB_ISFR |= 0x00000010; 
     printf("\n****PORTB4 ISR ISF cleared *****\r\n");

     } else  {
        printf("\n****PORTB4 ISR entered but ISF not set *****\r\n");
      } //end if else portb
}
void portc_isr(void)
{
  if(re_init_clk){
       mcg_pbe_2_pee();
 } 
  printf("\n\n****PORTC ISR entered*****\r\n");
  if ( PORTC_PCR1 & PORT_PCR_ISF_MASK){
     printf("\n****PORTC1 ISF FLAG CLEARED *****\r\n");
     PORTC_PCR1 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
  }
  if ( PORTC_PCR3 & PORT_PCR_ISF_MASK){
     printf("\n****PORTC3 ISF FLAG CLEARED *****\r\n");
     PORTC_PCR3 |= PORT_PCR_ISF_MASK ;    //  clear Flag if there
  }
  if ( PORTC_PCR5 & PORT_PCR_ISF_MASK){
     printf("\n****PORTC5 ISF FLAG CLEARED *****\r\n");
     PORTC_PCR5 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port C to clear interrupt flag
  }
  if ( PORTC_PCR8 & PORT_PCR_ISF_MASK){
     printf("\n****PORTC8 ISF FLAG CLEARED *****\r\n");
     PORTC_PCR8 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port C to clear interrupt flag
  }
  if ( PORTC_PCR13 & PORT_PCR_ISF_MASK){
     printf("\n****PORTC13 ISF FLAG CLEARED *****\r\n");
     PORTC_PCR13 |= PORT_PCR_ISF_MASK; //write 1 to ISF of port C to clear interrupt flag
  }
return;  
}

void portd_isr(void)
{
  if(re_init_clk){
       mcg_pbe_2_pee();
 } 
  printf("\n****PORTD ISR entered*****\r\n\n");
  if (PORTD_PCR0 & PORT_PCR_ISF_MASK) {
    printf("[PORTD_ISR] Interrput flag on port e pin 1 set \n");
    PORTD_PCR0 |= PORT_PCR_ISF_MASK; 
    //write 1 to ISF of port C pin 8 to clear interrupt flag
    }
    printf("PORTE_PCR1 register =   = %#02X \r\n",    (PORTE_PCR1 )) ;
return;  
}
void porte_isr(void)
{
  if(re_init_clk){
       mcg_pbe_2_pee();
 } 
  printf("\n****PORTE ISR entered*****\r\n\n");
  if (PORTE_PCR26 & PORT_PCR_ISF_MASK) {
    printf("[PORTE_ISR] Interrput flag on port E pin 26 set \n");
    PORTE_PCR26 |= PORT_PCR_ISF_MASK; 
    }
return;  
}

void rtc_isr(void)
{  
  if(re_init_clk){
      mcg_pbe_2_pee();
  } 
     if( seconds_count == 1) {
       sec = sec +1;
       
       if(sec ==10) {
       tsec = tsec +1;
       sec =0; 
       
       } 
   }
   if ((seconds_count == 60) | (tsec ==6)){
       mins = mins +1;
       tsec = 0;
       }
       if (mins == 10){
       tmins = tmins +1;
       mins =0;
       
       }
       if(tmins == 6){
       hour = hour +1;
       tmins = 0;
       }
       
       if(hour  == 10){
       thour = thour +1;
       hour  = 0;
       }
       
       if ((thour == 1)&&(hour ==2)){
       thour = 0;
       hour = 1 ;
       AMPM = !AMPM;
       }
   if(( seconds_count == 1) | (seconds_count == 60)) {
     printf(" [RTC isr] THR = %#02X ", (thour));
     printf(" HR = %#02X ", (hour));
     printf(" TM = %#02X ", (tmins));
     printf(" M = %#02X ", (mins));
     printf(" TSEC = %#02X ", (tsec));
     printf(" SEC = %#02X \r", (sec));
     } else {
       printf("\n[RTC isr] Current RTC_TAR  = %#02X ",  (RTC_TAR) )  ;
   }
  
  RTC_SR  |= RTC_SR_TAF_MASK; 
  RTC_TAR = RTC_TSR-1 + seconds_count; 
  if(( seconds_count != 1) & (seconds_count != 60)) 
  printf(" Next wakeup at RTC_TAR  = %#02X ",  (RTC_TAR) )  ;
  NVICICPR2|=(1<<2);   //Clear any pending interrupts on RTC
  RTC_SR |= RTC_SR_TCE_MASK | RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;
  RTC_TSR = 0x01;     
}

//******************************************************************************

void ftm2_isr(void)
{
  uint32 temp_reg;

  temp_reg = FTM2_SC;              // read FTM2_SC 
  temp_reg &= ~FTM_SC_TOF_MASK;   // Clear TOF
  FTM2_SC = temp_reg;
  GPIOC_PTOR = 0x00000080; // toggle port C 7      
  return;
}
