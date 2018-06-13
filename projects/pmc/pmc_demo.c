/********************************************************************************
*                                                                     
*       Copyright (C) 2010 Freescale Semiconductor, Inc.              
*       All Rights Reserved								              
*														              
* Filename:     pmc_demo.c                
*														              
* Revision:     1.0 	
*
* Author:       Philip Drake
*														              
* Functions:    Demo code for the K60 Tower board:
*               RD - displays the current PMC< MC and LLWU register values
*               WRT - allows a new value to be written to a register
*               Autotrim - allows the fast and slow IRCs to be trimmed using the ATM
*               RTC_EN - example code from the QRUG to use the 32 kHz RTC oscillator 
*                         as the reference clock for the FLL.
*         
* Description:  
*
* Notes:        Routines are included to allow the movement from any MCG mode to any
*               other although not necessarily in the shortest way. You may need to go
*               through an indirect path.
*
*
* Instructions:  Uses serial terminal command-line interface scheme established
*                within \common\uif.c.  Default baud is 19200.
*                The auto trim should be run first to trim both IRCs
*                The slow IRC should be trimmed to 32768 Hz (atc 0 32768)
*                The fast IRC shoud be trimmed to 3600000 Hz (atc 1 3600000)
*                Using other frequencies may cause the serial communication to fail due to the baud rate being wrong
*
*                Type 'help' to see a list of commands/descriptions.
*                Type desired command to execute individual validation test 
*                cases, or to read/write registers for manual configuration.
*********************************************************************************/
#include "common.h"
#include "vectors.h"
#include "pmc_demo.h"
#include "uif.h"
#include "uart.h"
#include "isr.h"
#include "pmc_demo.h"
#include "pmc.h"
#include "peripheral.h"


/********************************************************************/
/*
 * Setup user interface
 */
const char BANNER[] = "\nPMC Tower Demo\n";
const char PROMPT[] = "Test> ";
unsigned char state;
unsigned char clks_val;
uint32 TestPassesCount;
UIF_CMD UIF_CMDTAB[] =
{
    UIF_CMDS_ALL
    {"rd",0,0,0,cmd_rd, "MC, PMC, and LLWU Register Display",""},
    {"rd_mcg",0,0,0,cmd_rd_mcg, "MCG Register Register Display",""},
    {"wrt_mcg",0,3,0,cmd_wrt_mcg, "Write MCG Registers <register index> <value>"},
    {"wrt",0,2,0,cmd_wrt, "Write MC_LLWU_PMC  <register index> <value>"},
    {"set_en_rtc",0,3,0,set_en_rtc,  "Set_en_rtc <time out > <1=enable LLWU > <1=enable RTC>"},
    {"set_en_lpt",0,3,0,set_en_lpt,  "Set_en_lpt <compare value> <1=enable LLWU> <1=enable LPT>"},
    {"en_int",0,0,0,en_int,  "enable interrupts", ""},
    {"dis_int",0,0,0,dis_int,  "disable interrupts", ""},
    {"tst_stop",0,2,0,tst_stop_mode,   "Test stop mode  <restore pee clock mode>", ""},
    {"tst_wait",0,0,0,tst_wait_mode,   "Test wait mode ", ""},
    {"enter_vlpr",0,1,0,enter_vlpr_mode,   "Enter vlpr mode <1/0 LPWUI> ", ""},
    {"exit_vlpr",0,0,0,exit_vlpr_mode,   "Exit vlpr mode ", ""},
    {"tst_vlps",0,2,0,tst_vlps_mode,   "Test vlps mode <1/0 LPWUI> <1 = goto pee> <0 = clock same>", ""},
    {"tst_lls",0,2,0,tst_lls_mode,    "Test lls mode  <restore pee clock mode>", ""},
    {"tst_vlls1",0,0,0,tst_vlls1_mode,  "Test vlls1 mode  ", ""},
    {"tst_vlls2",0,0,0,tst_vlls2_mode,  "Test vlls2 mode  ", ""},
    {"tst_vlls3",0,0,0,tst_vlls3_mode,  "Test vlls3 mode  ", ""},
    {"atc",0,2,0,cmd_atc,"Autotrim <0 = slow, 1 = fast> <frequency in Hz>"},
 
};

UIF_SETCMD UIF_SETCMDTAB[] =
{
    {"sysclk", 0,1,setshow_sysclk,""},
};

const int UIF_NUM_CMD    = UIF_CMDTAB_SIZE;
const int UIF_NUM_SETCMD = UIF_SETCMDTAB_SIZE;

//global variables
unsigned char seconds_count= 1;
char DISPLAY_SEC;
volatile unsigned char sec = 5;
volatile unsigned char tsec = 5;
volatile unsigned char mins = 9;
volatile unsigned char tmins = 5;
volatile unsigned char hour = 2;
volatile unsigned char thour= 1;
volatile unsigned char AMPM = 0;  //0 = AM 1 = PM
volatile unsigned char clock_on;  //1 = Clock displayed on LCD 0 - not
int clock_on_off;
int simclkdiv;

int  error_cnt=0;
int *res_ptr;
int  redge_flag=0;    // Flag to indicate when Rising Edge Interrupt has occurred.
int  fedge_flag=0;    // Flag to indicate when Falling Edge Interrupt has occurred.
int  index=0;         // Looping index used to select which of the three CMPs to test.
int  re_init_clk;

extern int core_clk_khz;
extern int core_clk_mhz;
extern int periph_clk_khz;
unsigned char sctrim_val;
unsigned char scftrim_val;

char *dummy_argv[10];
char dummyline [10];
CMP_MemMapPtr cmpch;



void main(void)
{
  
//    DisableInterrupts;
  
/* Enable FTM2 Interrupt in NVIC*/
    sim_clk_gates_set();
    Port_Init();
    SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;
 
    if(RTC_CR & RTC_CR_SWR_MASK){
    RTC_CR &= ~RTC_CR_SWR_MASK;   // Clear SWR MASK
    }   
    if (RTC_SR & RTC_SR_TIF_MASK){
        RTC_TSR = 0x00000000;   //  this action clears the TIF
        printf("RTC Invalid flag was set - Write to TSR done to clears RTC_SR =  %#02X \n",  (RTC_SR) )  ;
    }

    Int_Vector_Set();
    re_init_clk =0;
    EnableInterrupts;

    //ftm_enable();
    //timer_setup();
    // autotrim the slow and fast clock in mcg while in pee
    dummy_argv[1] = "0";
    dummy_argv[2] = "32000";
    cmd_atc(3, dummy_argv);
    dummy_argv[1] = "1"; 
    dummy_argv[2] = "3600000";    
    cmd_atc(3, dummy_argv);
    

    printf("\n");
    printf("**************************************************\n");
    printf("*                                                *\n");
    printf("* PMC, LLWU and MC Tower Demo   vlpr_version 1   *\n");
    printf("*                                                *\n");
    printf("**************************************************\n");
    printf(HELPMSG);
    printf("\n\n");

//    outSRS();
    mainloop();
}// end main

//******************************************************************************
void mainloop (void)
{

    while (TRUE)
    {
        printf(PROMPT);
        run_cmd();
    }
}// end mainloop
   
//******************************************************************************
//***************************  TEST CASES  *************************************
//******************************************************************************
void cmd_rd (int argc, char **argv)
{

    printf("\n-------------------------------------------\n");
    printf("       PMC MC LLWU Register Read Utility\n");
    printf("-------------------------------------------\n");
    printf("\n");
        printf("PMC LVDSC1 = %#02X \r\n", (PMC_LVDSC1));
        printf("PMC LVDSC2 = %#02X \r\n", (PMC_LVDSC2));
        printf("PMC REGSC = %#02X \r\n\n",(PMC_REGSC) );
        printf("MC SRSH = %#02X \r\n",    (MC_SRSH)   ) ;
        printf("MC SRSL = %#02X \r\n",    (MC_SRSL)    );
        printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
        printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
        printf("LLWU PE1 = %#02X \r\n",   (LLWU_PE1)) ;
        printf("LLWU PE2 = %#02X \r\n",   (LLWU_PE2) );
        printf("LLWU PE3 = %#02X \r\n",   (LLWU_PE3)) ;
        printf("LLWU PE4 = %#02X \r\n\n", (LLWU_PE4) );
        printf("LLWU ME = %#02X \r\n",    (LLWU_ME) ) ;
        printf("LLWU F1 = %#02X \r\n",    (LLWU_F1) ) ;
        printf("LLWU F2 = %#02X \r\n",    (LLWU_F2) ) ;
        printf("LLWU F3 = %#02X \r\n",    (LLWU_F3) ) ;
        printf("LLWU CS = %#02X \r\n\n",  (LLWU_CS) ) ;
   
}// end cmd_rd

//******************************************************************************

void
cmd_wrt (int argc, char **argv)
{
    uint32 regindex;
    uint32 regvalue;
    
    int result = 0;
    res_ptr = &result;
         
    printf("\n-------------------------------------------\n");
    printf("  PMC MC LLWU Single Register Write Utility\n");
    printf("-------------------------------------------\n");
    printf("\n");
    if (argc != 3)
    {
        printf("Valid 'wrt' syntax:\n");
        printf("PMC> wrt  <register index> <value> \n\n");
        printf("PMC_MC_LLWU register index and current values are: \n\n");
        printf("PMC LVDSC1 Register  0 = %#02X \r\n",    (PMC_LVDSC1))  ;
        printf("PMC LVDSC2 Register  1 = %#02X \r\n",    (PMC_LVDSC2))  ;
        printf("PMC REGSC  Register  2 = %#02X \r\n\n",  (PMC_REGSC) )  ;
        printf("PMC SRSH   Register  3 = %#02X \r\n",    (MC_SRSH)   )   ;
        printf("MC SRSL    Register  4 = %#02X \r\n",    (MC_SRSL)   )   ;
        printf("MC PMPROT  Register  5 = %#02X \r\n",    (MC_PMPROT) ) ; 
        printf("MC PMCTRL  Register  6 = %#02X \r\n\n",  (MC_PMCTRL) )   ;
        printf("LLWU PE1   Register  7 = %#02X \r\n",    (LLWU_PE1)  ) ;
        printf("LLWU PE2   Register  8 = %#02X \r\n",    (LLWU_PE2)  ) ;
        printf("LLWU PE3   Register  9 = %#02X \r\n",    (LLWU_PE3)  ) ;
        printf("LLWU PE4   Register 10 = %#02X \r\n\n",  (LLWU_PE4)  ) ;
        printf("LLWU ME    Register 11 = %#02X \r\n",    (LLWU_ME)  ); 
        printf("LLWU F1    Register 12 = %#02X \r\n",    (LLWU_F1)  )  ;
        printf("LLWU F2    Register 13 = %#02X \r\n",    (LLWU_F2)  )  ;
        printf("LLWU F3    Register 14 = %#02X \r\n",    (LLWU_F3)  )  ;
        printf("LLWU CS    Register 15 = %#02X \r\n\n",  (LLWU_CS)  )  ;

        return;
    }
    else{
      
        regindex = get_value(argv[1],res_ptr,10);
        regvalue = get_value(argv[2],res_ptr,16);
                
         if(regvalue<256){        
              switch (regindex){
                 case 0: PMC_LVDSC1 = regvalue;
                    break;
                 case 1: PMC_LVDSC2 = regvalue;
                    break;
                 case 2: PMC_REGSC = regvalue;
                    break;
                 case 3: MC_SRSH = regvalue;
                    break;
                 case 4: MC_SRSL = regvalue;
                    break;
                 case 5: MC_PMPROT = regvalue;
                    break;
                 case 6: MC_PMCTRL = regvalue;
                    break;
                 case 7: LLWU_PE1 = regvalue;
                    break;
                 case 8: LLWU_PE2 = regvalue;
                    break;
                 case 9: LLWU_PE3 = regvalue;
                    break;
                 case 10: LLWU_PE4 = regvalue;
                    break;
                 case 11: LLWU_ME = regvalue;
                    break;
                 case 12: LLWU_F1 = regvalue;
                    break;
                 case 13: LLWU_F2 = regvalue;
                    break;
                 case 14: LLWU_F3 = regvalue;
                    break;
                 case 15: LLWU_CS = regvalue;
                    break;
                 default: break;
              } // end switch/case regindex
         } // if
        else 
        printf("Register value not in valid range (0x00-0xFF).\r\n\n");
        printf("PMC LVDSC1 Register  0 = %#02X \r\n",    (PMC_LVDSC1))  ;
        printf("PMC LVDSC2 Register  1 = %#02X \r\n",    (PMC_LVDSC2))  ;
        printf("PMC REGSC  Register  2 = %#02X \r\n\n",  (PMC_REGSC) )  ;
        printf("MC SRSH   Register  3 = %#02X \r\n",    (MC_SRSH)   )   ;
        printf("MC SRSL    Register  4 = %#02X \r\n",    (MC_SRSL)   )   ;
        printf("MC PMPROT  Register  5 = %#02X \r\n",    (MC_PMPROT) ) ; 
        printf("MC PMCTRL  Register  6 = %#02X \r\n\n",  (MC_PMCTRL) )   ;
        printf("LLWU PE1   Register  7 = %#02X \r\n",    (LLWU_PE1)  ) ;
        printf("LLWU PE2   Register  8 = %#02X \r\n",    (LLWU_PE2)  ) ;
        printf("LLWU PE3   Register  9 = %#02X \r\n",    (LLWU_PE3)  ) ;
        printf("LLWU PE4   Register 10 = %#02X \r\n\n",  (LLWU_PE4)  ) ;
        printf("LLWU ME    Register 11 = %#02X \r\n",    (LLWU_ME)  ); 
        printf("LLWU F1    Register 12 = %#02X \r\n",    (LLWU_F1)  )  ;
        printf("LLWU F2    Register 13 = %#02X \r\n",    (LLWU_F2)  )  ;
        printf("LLWU F3    Register 14 = %#02X \r\n",    (LLWU_F3)  )  ;
        printf("LLWU CS    Register 15 = %#02X \r\n\n",  (LLWU_CS)  )  ;
    }// end else
}// end cmd_wrt

void cmd_rd_mcg (int argc, char **argv)
{

    printf("\n-------------------------------------------\n");
    printf("       MCG Register Read Utility\n");
    printf("-------------------------------------------\n");
    printf("\n");
        printf("MCG_C1   = %#02X \r\n", (MCG_C1));
        printf("MCG_C2   = %#02X \r\n", (MCG_C2));
        printf("MCG_C3   = %#02X \r\n", (MCG_C3));
        printf("MCG_C4   = %#02X \r\n", (MCG_C4)) ;
        printf("MCG_C5   = %#02X \r\n", (MCG_C5));
        printf("MCG_C6   = %#02X \r\n\n", (MCG_C6));
        printf("MCG_S    = %#02X \r\n\n", (MCG_S)) ;
        printf("MCG_ATC   = %#02X \r\n",   (MCG_ATC)) ;
        printf("MCG_ATCVL = %#02X \r\n",   (MCG_ATCVL)) ;
        printf("MCG_ATVCH = %#02X \r\n",   (MCG_ATCVH));
   
}// end cmd_rd

//******************************************************************************

void
cmd_wrt_mcg (int argc, char **argv)
{
    uint32 regindex;
    uint32 regvalue;
    
    int result = 0;
    res_ptr = &result;
         
    printf("\n-------------------------------------------\n");
    printf("  MCG Single Register Write Utility\n");
    printf("-------------------------------------------\n");
    printf("\n");
    if (argc != 3)
    {
        printf("Valid 'wrt' syntax:\n");
        printf("MCG> wrt  <register index> <value> \n\n");
        printf("MCG register index and current values are: \n\n");
        printf("MCG_C1     Register  0 = %#02X \r\n",    (MCG_C1))  ;
        printf("MCG_C2     Register  1 = %#02X \r\n",    (MCG_C2))  ;
        printf("MCG_C3     Register  2 = %#02X \r\n",  (MCG_C3) )  ;
        printf("MCG_C4     Register  3 = %#02X \r\n",    (MCG_C4)   )   ;
        printf("MCG_C5     Register  4 = %#02X \r\n",    (MCG_C5)   )   ;
        printf("MCG_C6     Register  5 = %#02X \r\n\n",    (MCG_C6) ) ; 
        printf("MCG_S      Register  6 = %#02X \r\n\n",  (MCG_S) )   ;
        printf("MCG_ATC     Register  7 = %#02X \r\n",    (MCG_ATC)  ) ;
        printf("MCG_ATCVL   Register  8 = %#02X \r\n",    (MCG_ATCVL)  ) ;
        printf("MCG_ATCVH   Register  9 = %#02X \r\n",    (MCG_ATCVH)  ) ;

        return;
    }
    else{
      
        regindex = get_value(argv[1],res_ptr,10);
        regvalue = get_value(argv[2],res_ptr,16);
                
         if(regvalue<256){        
              switch (regindex){
                 case 0: MCG_C1 = regvalue;
                    break;
                 case 1: MCG_C2 = regvalue;
                    break;
                 case 2: MCG_C3 = regvalue;
                    break;
                 case 3: MCG_C4 = regvalue;
                    break;
                 case 4: MCG_C5 = regvalue;
                    break;
                 case 5: MCG_C6 = regvalue;
                    break;
                 case 6: MCG_S = regvalue;
                    break;
                 case 7: MCG_ATC = regvalue;
                    break;                      
                 case 8: MCG_ATCVL = regvalue;
                    break;
                 case 9: MCG_ATCVH = regvalue;
                    break;
                 default: break;
              } // end switch/case regindex
         } // if
        else 
        printf("Register value not in valid range (0x00-0xFF).\r\n\n");
        printf("MCG_C1     Register  0 = %#02X \r\n",    (MCG_C1))  ;
        printf("MCG_C2     Register  1 = %#02X \r\n",    (MCG_C2))  ;
        printf("MCG_C3     Register  2 = %#02X \r\n\n",  (MCG_C3) )  ;
        printf("MCG_C4     Register  3 = %#02X \r\n",    (MCG_C4)   )   ;
        printf("MCG_C5     Register  4 = %#02X \r\n",    (MCG_C5)   )   ;
        printf("MCG_C6     Register  5 = %#02X \r\n",    (MCG_C6) ) ; 
        printf("MCG_S      Register  6 = %#02X \r\n\n",  (MCG_S) )   ;
        printf("MCG_ATC    Register  7 = %#02X \r\n",    (MCG_ATC)  ) ;
        printf("MCG_ATCVL   Register  8 = %#02X \r\n",    (MCG_ATCVL)  ) ;
        printf("MCG_ATCVH   Register  9 = %#02X \r\n",    (MCG_ATCVH)  ) ;
    }// end else
}// end cmd_wrt


void cmd_atc (int argc, char **argv)
{
    unsigned char regindex;
    uint32 regvalue;
    uint32 temp_reg = 0;
    unsigned char temp_reg8 = 0;
    unsigned char irclken_state;
    uint32 orig_SIM_CLKDIV1;
    unsigned short atcv;
    
    int result = 0;
    res_ptr = &result;
        
     if (argc != 3)
    {
        printf("Valid 'atc' syntax:\n");
        printf("MCG> atc  <0 = slow, 1 = fast> <frequency in Hz> \n");

        return;
    }
    else{
      
        regindex = get_value(argv[1],res_ptr,10);
        regvalue = get_value(argv[2],res_ptr,10);
        clks_val = ((MCG_C1 & MCG_C1_CLKS_MASK) >> 6);
        
        if (((clks_val == 1) || (clks_val ==3)))
        {
          printf("\nAuto trim error - cannot use the internal clock source.\r\n");
          return; //error using IRC as system clock
        }
              
        if ((clks_val == 0) && (!(MCG_C6 & MCG_C6_PLLS_MASK)) && (MCG_C1 & MCG_C1_IREFS_MASK))
        {
          printf("\nAuto trim error - cannot use the FLL with internal clock source.\r\n");
          return; // error using FLL with IRC
        }
        if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 3)
        {
          printf("\nWARNING - PLL is not the internal clock source. Auto trim value will not be correct\r\n\n");
        }
                
        if(!regindex) //determine if slow or fast IRC to be trimmed
        {
          // NOTE: IRCLKEN should not be required - 1.0 errata.
          // Remove this when silicon is fixed.         
          MCG_C2 &= ~MCG_C2_IRCS_MASK; //must select the slow IRC to be enabled
          // store MCG_C1 value to maintain IRCLEN state after autotrim is finished
          irclken_state = MCG_C1;
          MCG_C1 |= MCG_C1_IRCLKEN_MASK; // set IRCLKEN
        } // if
        orig_SIM_CLKDIV1 = SIM_CLKDIV1;        // backup current divider value
        temp_reg = SIM_CLKDIV1;                // bus clock needs to be between 8 and 16 MHz
        temp_reg &= ~SIM_CLKDIV1_OUTDIV2_MASK; // clear bus divider field
        temp_reg |= SIM_CLKDIV1_OUTDIV2(5);    //divide 96MHz PLL by 6 = 16MHz bus clock
        SIM_CLKDIV1 = temp_reg; 
// Set up autocal registers, must use floating point calculation
        if (regindex)
          atcv = (unsigned short)(128.0f * (21.0f * (16000000.0f / (float)regvalue))); 
        else
          atcv = (unsigned short)(21.0f * (16000000.0f / (float)regvalue));
        
        MCG_ATCVL = (atcv & 0xFF); //Set ATCVL to lower 8 bits of count value
        MCG_ATCVH = ((atcv & 0xFF00) >> 8); // Set ATCVH to upper 8 bits of count value

// Enable autocal
        MCG_ATC = 0x0; // clear auto trim control register
        temp_reg8 |= (MCG_ATC_ATME_MASK | (regindex << MCG_ATC_ATMS_SHIFT)); //Select IRC to trim and enable trim machine
        MCG_ATC = temp_reg8;
        
        while (MCG_ATC & MCG_ATC_ATME_MASK) {}; //poll for ATME bit to clear
        
        SIM_CLKDIV1 = orig_SIM_CLKDIV1; //restore the divider value
        
        if (MCG_ATC & MCG_ATC_ATMF_MASK) // check if error flag set
        {
          printf("Autotrim error.\r\n\n");
          printf("\n");
          printf("MCG_C1   = %#02X \r\n", (MCG_C1));
          printf("MCG_C2   = %#02X \r\n", (MCG_C2));
          printf("MCG_C3   = %#02X \r\n", (MCG_C3));
          printf("MCG_C4   = %#02X \r\n", (MCG_C4)) ;
          printf("MCG_C5   = %#02X \r\n", (MCG_C5));
          printf("MCG_C6   = %#02X \r\n\n", (MCG_C6));
          printf("MCG_S    = %#02X \r\n\n", (MCG_S)) ;
          printf("MCG_ATC   = %#02X \r\n",   (MCG_ATC)) ;
          printf("MCG_ATCVL = %#02X \r\n",   (MCG_ATCVL)) ;
          printf("MCG_ATVCH = %#02X \r\n",   (MCG_ATCVH));
          MCG_ATC |= MCG_ATC_ATMF_MASK; // clear fail flag
        } 
        else 
        {      
          printf("Autotrim Passed.\r\n");
          printf("MCG_C3   = %#02X \r\n", (MCG_C3));
          printf("MCG_C4   = %#02X \r\n", (MCG_C4));
          // Check trim value is not at either extreme of the range
          if (!regindex)
          {
            sctrim_val = MCG_C3; //store slow IRC trim value
            scftrim_val = MCG_C4 & MCG_C4_SCFTRIM_MASK; // store fine trim bit
            if ((MCG_C3 == 0xFF) || (MCG_C3 == 0))
            {
              printf("\nAutotrim result is not valid.\r\n\n");
            }
            // restore MCG_C1 value 
            MCG_C1 = irclken_state;
          }
          else
          {
            if ((((MCG_C4 & MCG_C4_FCTRIM_MASK) >> MCG_C4_FCTRIM_SHIFT) == 0xF) ||
              (((MCG_C4 & MCG_C4_FCTRIM_MASK) >> MCG_C4_FCTRIM_SHIFT) == 0))
            {
              printf("\nAutotrim result is not valid.\r\n\n");
            }
          }
        }
          
    }// end else
}// end cmd_atc

   
void set_en_rtc (int argc, char **argv)
{
    int i;
    uint32 llwu_w_rtc, rtc_enabled;
    int result = 0;
    res_ptr = &result;

    

    printf("\n--------------------------------------------------\n");
    printf("  RTC Set and Enable Utility \n");
    printf("--------------------------------------------------\n");

    if (argc != 4){
        printf("Valid 'set_en_rtc' syntax:\n");
        printf("set_en_rtc <time out sec count> <1 = enable LLWU wakeup> <1 = Enable RTC> \n");
        printf("RTC register values are: \n\n");
        printf("RTC_TSR  = %#02X \r\n",    (RTC_TSR ))  ;
        printf("RTC_TPR  = %#02X \r\n",    (RTC_TPR ))  ;
        printf("RTC_TAR  = %#02X \r\n",  (RTC_TAR) )  ;
        printf("RTC_TCR  = %#02X \r\n",    (RTC_TCR ))  ;
        printf("RTC_CR   = %#02X \r\n",    (RTC_CR ))  ;
        printf("RTC_SR   = %#02X \r\n",  (RTC_SR) )  ;
        printf("RTC_LR   = %#02X \r\n",    (RTC_LR ))  ;
        printf("RTC_IER  = %#02X \r\n",    (RTC_IER ))  ;
        printf("RTC_WAR  = %#02X \r\n",  (RTC_WAR) )  ;
        printf("RTC_RAR  = %#02X \r\n\n",  (RTC_RAR) )  ;
        return;
    }
    else{
       seconds_count = get_value(argv[1],res_ptr,16);
       llwu_w_rtc = get_value(argv[2],res_ptr,16);
       rtc_enabled = get_value(argv[3],res_ptr,16);

       //if (MC_SRSL & MC_SRSL_POR_MASK)
       RTC_Init();
       if (rtc_enabled == 1){
         // Enable the timer counter and clear the flags
         RTC_SR |= RTC_SR_TCE_MASK | RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;
         RTC_TAR = RTC_TSR + seconds_count;
         printf("RTC_TAR  = %#02X \r\n\n",  (RTC_TAR) )  ;
       }else{
         RTC_SR &= ~RTC_SR_TCE_MASK ;
         // clear the flags
         RTC_SR |=  RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;   
         RTC_TAR = 0x00;  // set this register to other than 01 so set TAF
         printf("RTC_TAR  = %#02X \r\n\n",  (RTC_TAR) )  ;
       }
       if (llwu_w_rtc ==0){
           LLWU_ME &= ~LLWU_ME_WUME5_MASK ; // disable the RTC to wake up from low power modes
           printf("RTC LLWU disabled; LLWU ME  = %#02X \r\n",    (LLWU_ME)  );  

       } else {
             // Enable the timer counter and clear the flags
             RTC_SR |= RTC_SR_TCE_MASK | RTC_SR_TAF_MASK |RTC_SR_TOF_MASK|RTC_SR_TIF_MASK;
             RTC_TAR = RTC_TSR + seconds_count;
             LLWU_ME |= LLWU_ME_WUME5_MASK ; // enable the RTC to wake up from low power modes
             printf("RTC LLWU enabled;  LLWU ME  = %#02X \r\n",    (LLWU_ME)  );
        } // end else

       printf("Hit any key to stop testing \n");

       for (i= 0;i<0xffffffff;i++){
          printf("TSR = %#02X ",    (RTC_TSR ))  ;
          printf(" TAR = %#02X ",    (RTC_TAR ))  ;
          printf(" TPR = %#02X \r",    (RTC_TPR ))  ;
          //if character has been received exit //
          if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
              break;
          } //if char received  */
       }//	for
     }
}
//******************************************************************************
   
void set_en_lpt (int argc, char **argv)
{
    int i;
    int compare_value;
    uint32 llwu_w_lpt, lpt_enabled;
    int result = 0;
    res_ptr = &result;

    printf("\n--------------------------------------------------\n");
    printf("  LPT Set and Enable Utility \n");
    printf("--------------------------------------------------\n");

    if (argc != 4){
        printf("Valid 'set_en_lpt' syntax:\n");
        printf("set_en_lpt <compare value> <1 = enable LLWU wakeup> <1 = Enable LPT> \n");
        printf("LPT register values are: \n\n");
        printf("LPTMR0_CMR  = %#02X \r\n",    (LPTMR0_CMR ))  ;
        printf("LPTMR0_PSR  = %#02X \r\n",    (LPTMR0_PSR ))  ;
        printf("LPTMR0_CSR  = %#02X \r\n",  (LPTMR0_CSR) )  ;
        return;
    }
    else{
       compare_value = get_value(argv[1],res_ptr,16);
       llwu_w_lpt = get_value(argv[2],res_ptr,16);
       lpt_enabled = get_value(argv[3],res_ptr,16);

 
       if (lpt_enabled == 1){
         // Enable the timer counter and clear the flags
          LPT_Init(compare_value);
          printf("LPTMR0_CMR  = %#02X \r\n",    (LPTMR0_CMR ))  ;
          printf("LPTMR0_PSR  = %#02X \r\n",    (LPTMR0_PSR ))  ;
          printf("LPTMR0_CSR  = %#02X \r\n",  (LPTMR0_CSR) )  ;
          printf("LPT Enabled \n\r");
       }else{
          /* Reset LPT Module */
          clear_lpt_registers();
          printf("LPTMR0_CMR  = %#02X \r\n",    (LPTMR0_CMR ))  ;
          printf("LPTMR0_PSR  = %#02X \r\n",    (LPTMR0_PSR ))  ;
          printf("LPTMR0_CSR  = %#02X \r\n",  (LPTMR0_CSR) )  ;
          NVICICER2|=(1<<21);   //Clear any pending interrupts on LPT
          NVICICPR1|=(0<<21);   //Clear any pending interrupts on LLWU
          printf("LPT Disabled \n\r");
       }
       if (llwu_w_lpt ==0){
           LLWU_ME &= ~LLWU_ME_WUME0_MASK ; // disable the LPT to wake up from low power modes
           printf("LPT LLWU disabled; LLWU ME  = %#02X \r\n",    (LLWU_ME)  );  
           NVICICPR1|=(0<<21);   //Clear any pending interrupts on LLWU

       } else {
             // Enable the timer counter and clear the flags
             LPT_Init(compare_value);
             LLWU_ME |= LLWU_ME_WUME0_MASK ; // enable the LPT to wake up from low power modes
             printf("LPT LLWU enabled;  LLWU ME  = %#02X \r\n",    (LLWU_ME)  );
        } // end else

       printf("Hit any key to stop testing \n");

       for (i= 0;i<0xfffffff;i++){
          LPTMR0_CSR|= LPTMR_CSR_TEN_MASK; //Turn on LPT and start counting

          printf("LPTMR0_CMR  = %#02X  ",    (LPTMR0_CMR ))  ;
          printf("LPTMR0_PSR  = %#02X  ",    (LPTMR0_PSR ))  ;
          printf("LPTMR0_CSR  = %#02X \r",  (LPTMR0_CSR) )  ;
          //if character has been received exit //
          if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
              break;
          } //if char received  */
          LPTMR0_CSR &=~LPTMR_CSR_TEN_MASK; //Turn off LPT to avoid more interrupts
 
       }//	for
     }
} //set_en_lpt
//******************************************************************************
//******************************************************************************

void tst_stop_mode (int argc, char **argv)
{

    int result = 0;
    int i;  
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter STOP Utility \n");
    printf("--------------------------------------\n");
    if (argc != 2){
        printf("Valid 'tst_stop' syntax:\n");
        printf("tst_stop <1 = restore pee 0 = same clock mode> \n\n");
        return;
    }
    else{
      re_init_clk = get_value(argv[1],res_ptr,16);       // argument that re-initialized clocks.

      MC_PMPROT = MC_PMPROT_AVLP_MASK;  // write oneif not all set make sure all enabled
                                    //this write-once bit allows the MCU to enter the
                                    //very low power modes: VLPR, VLPW, and VLPS.

    MC_PMCTRL = MC_PMCTRL_LPWUI_MASK | MC_PMCTRL_LPLLSM(0) ;  
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
   
    printf("Stop mode Wakeup 8 times \r\n\n"  );
    for (i= 0;i<0x08;i++)
    {      
          //       stop();
          /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
        	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
          //GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	    /* WFI instruction will start entry into STOP mode */
    	    asm("WFI");
          printf("\nMC STOP Loop Cnt %#02X \r\n\n",   (i)  );

          //if character has been received exit //
          if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
            break;
          } //if char received  */
    } //	for
    
    printf("  MC_Exited NORMAL STOP  \n");
    }
    re_init_clk = 0;
   /*
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */    
}  //end tst_stop_mode
//******************************************************************************

void tst_wait_mode (int argc, char **argv)
{
    int result = 0;
    int i;  
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter Wait Utility \n");
    printf("--------------------------------------\n");
    
    MC_PMCTRL = MC_PMCTRL_LPWUI_MASK  ;  
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
     
    printf("Wait mode Wakeup 8 times \r\n\n" );

    for (i= 0;i<0x08;i++){
        
        wait();
          
        printf("MC WAIT Loop Cnt %#02X \r\n",   (i)  );
        //if character has been received exit //
        if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
            break;
        } //if char received  */
    }//	for
    
    printf("MC_Exited WAIT \r\n");
   
  
}
//******************************************************************************
 
void enter_vlpr_mode (int argc, char **argv)
{
    int setclr_lpwui;
    int result = 0;
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC Enter VLPR  Utility \n");
    printf("--------------------------------------\n");
    if (argc != 2){
        printf("Valid 'enter_vlpr' syntax:\n");
        printf("enter_vlpr  <LPWUI value> ");
    }else{
       setclr_lpwui = get_value(argv[1],res_ptr,16);
       enter_vlpr(setclr_lpwui);
    }
}
void exit_vlpr_mode (int argc, char **argv)
{
    printf("\n-------------------------------------\n");
    printf("  MC Directed Exit from VLPR  Utility \n");
    printf("--------------------------------------\n");
 
    MC_PMCTRL &= ~(MC_PMCTRL_RUNM(0x3));  // Clear RUNM=%00.
         
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
    while (PMC_REGSC & PMC_REGSC_VLPRS_MASK) {           // 0 MCU is not in VLPR mode
                                                         // 1 MCU is in VLPR mode
       printf("  - hit a key to continue. ");
       printf("PMC_REGSC = %#02X \r", (PMC_REGSC) ) ;
       if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
           break;
       }//if char received  */
    } //while VLPRS set - still in low power run mode.
    if (PMC_REGSC & PMC_REGSC_VLPRS_MASK) {
       printf("\n                Still in VLPR mode  - PMC_REGSC = %#02X \r\n", (PMC_REGSC) ) ;
    } else{
      
       printf("\n               Communicating in Run\n\r");


       SIM_CLKDIV1 =  simclkdiv;

       mcg_blpi_2_pee();
       core_clk_khz = core_clk_mhz * 1000;
       periph_clk_khz = core_clk_khz / (((SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> 24)+ 1);
       if ((TERM_PORT == UART0_BASE_PTR) | (TERM_PORT == UART1_BASE_PTR))
           uart_init (TERM_PORT, core_clk_khz, TERMINAL_BAUD);
       else
       uart_init (TERM_PORT, periph_clk_khz, TERMINAL_BAUD);
    
       printf("               Now in Run Mode at 19200 baud \r");
    }

}
//******************************************************************************

void tst_vlps_mode (int argc, char **argv)
{
    int result = 0;
    uint8 lpwui_value;
    int i;  
    res_ptr = &result;

    printf("\n-------------------------------------\n");
    printf("  MC_Enter VLPS Utility \n");
    printf("--------------------------------------\n");
    if (argc != 3){
        printf("Valid 'tst_vlps' syntax:\n");
        printf("tst_vlps <1/0 LPWUI_VALUE> <1 = restore pee, 0 = no clock mode change> \n\n");
        return;
    }
    else{
        re_init_clk = get_value(argv[2],res_ptr,16);       // argument that re-initialized clocks.
        lpwui_value = get_value(argv[1],res_ptr,16);       // argument that 
    
    MC_PMPROT = MC_PMPROT_AVLP_MASK;  // write oneif not all set make sure all enabled
                                    //this write-once bit allows the MCU to enter the
                                    //very low power modes: VLPR, VLPW, and VLPS.
    if (lpwui_value)
    MC_PMCTRL = MC_PMCTRL_LPWUI_MASK | MC_PMCTRL_LPLLSM(2) ;  //set LPWUI
    else
    MC_PMCTRL = MC_PMCTRL_LPLLSM(2) ;                    // clear LPWUI
    
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
       
    printf("VLPS mode Wakeup 8 times \r\n" );
    
    for (i= 0;i<0x08;i++){
 
        //       stop();
        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
      	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
        GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	  /* WFI instruction will start entry into STOP mode */
    	  asm("WFI");

        printf("MC_Exited VLPS Loop Cnt %#02X \r\n",   (i)  );
        //if character has been received exit //
        if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
          break;
        } //if char received  */
    } //	for
    
    printf("  MC_Exited VLPS  \n");
    }
    re_init_clk = 0;
   /*
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */
   
}
//******************************************************************************

void tst_lls_mode (int argc, char **argv)
{
    int i;  
    int result = 0;
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter LLS Utility \n");
    printf("--------------------------------------\n");
    if (argc != 2){
        printf("Valid 'tst_lls' syntax:\n");
        printf("tst_lls <1 = restore pee, 0 = same clock mode> \n\n");
        return;
    }
    else{
       MC_PMPROT = MC_PMPROT_ALLS_MASK;  // write oneif not all set make sure all enabled
                                       //this write-once bit allows the MCU to enter the
                                       //very low power modes: VLPR, VLPW, and VLPS.
       MC_PMCTRL = MC_PMCTRL_LPLLSM(3) ;  
       printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
       printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
  
       re_init_clk = get_value(argv[1],res_ptr,16);       // argument that re-initialized clocks.

        printf("\nLLS mode Wakeup 8 times \r\n"  );
      
       for (i= 0;i<0x08;i++){
           
          //       stop();
          /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
        	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
           GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	     /* WFI instruction will start entry into STOP mode */
    	     asm("WFI");
           printf("MC_Exited LLS Loop Cnt %#02X \r\n",   (i)  );
            
           //if character has been received exit //
           if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
          break;
           } //if char received  */
       } //	for
   
        printf("\n  MC_Exited LLS  \n");
      
      /*
       000 Normal stop
       001 Reserved
       010 Very low power stop (VLPS)
       011 Low leakage stop (LLS)
       100 Reserved
       101 Very low leakage stop 3 (VLLS3)
       110 Very low leakage stop 2 (VLLS2)
       111 Very low leakage stop 1 (VLLS1)
   */
    }
}
   //******************************************************************************

void tst_vlls1_mode (int argc, char **argv)
{
    int i;  
    int result = 0;
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter VLLS1 Utility \n");
    printf("--------------------------------------\n");
    
    MC_PMPROT = MC_PMPROT_AVLLS1_MASK;  // write oneif not all set make sure all enabled
                                    //this write-once bit allows the MCU to enter the
                                    //very low power modes: VLPR, VLPW, and VLPS.
    MC_PMCTRL = MC_PMCTRL_LPLLSM(7) ;  
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
      printf("\nMCU will wakeup from VLLS1 mode through reset\r\n"  );
 //make sure LED off
      GPIOC_PSOR = 0x00000080; // set port C 7      

    for (i= 0;i<0x10;i++){
        //       stop();
        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
      	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
        GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	  /* WFI instruction will start entry into STOP mode */
    	  asm("WFI");
        // should never reach this code - recovery through reset.
        printf("\nMC_Exited VLLS1 Loop ERROR !!!! %#02X \r\n",   (i)  );
        
        //if character has been received exit //
        if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
          break;
        } //if char received  */
    } //	for

    printf("  MC_Exited VLLS1  \n");
    re_init_clk = 0;   
   /*
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */
   
}
//******************************************************************************

void tst_vlls2_mode (int argc, char **argv)
{
    int i;  
    int result = 0;
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter VLLS2 Utility \n");
    printf("--------------------------------------\n");
      
    MC_PMPROT = MC_PMPROT_AVLLS2_MASK;  // write oneif not all set make sure all enabled
                                    //this write-once bit allows the MCU to enter the
                                    //very low power modes: VLPR, VLPW, and VLPS.
    MC_PMCTRL = MC_PMCTRL_LPLLSM(6) ;  
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
 
    printf("\nMCU will wakeup from VLLS2 mode through reset \r\n" );
    
    for (i= 0;i<0x10;i++){
        //       stop();
        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
      	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
        GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	  /* WFI instruction will start entry into STOP mode */
    	  asm("WFI");
        // should never reach this code - recovery through reset.

        printf("\nMC_Exited VLLS2 Loop ERROR !!!! %#02X \r\n",   (i)  );
        
        //if character has been received exit //
        if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
          break;
        } //if char received  */
    } //	for
   
    printf("  MC_Exited VLLS2  \n");
    re_init_clk = 0;
   /*
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */
   
}
//******************************************************************************

void tst_vlls3_mode (int argc, char **argv)
{
    int i;  
    int result = 0;
    res_ptr = &result;
    
    printf("\n-------------------------------------\n");
    printf("  MC_Enter VLLS3 Utility \n");
    printf("--------------------------------------\n");

    MC_PMPROT = MC_PMPROT_AVLLS3_MASK;  // write oneif not all set make sure all enabled
                                    //this write-once bit allows the MCU to enter the
                                    //very low power modes: VLPR, VLPW, and VLPS.
    MC_PMCTRL = MC_PMCTRL_LPLLSM(5) ;  
    printf("MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
    printf("MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
    printf("\nMCU will wakeup from VLLS3 mode through reset \r\n"  );
 
    for (i= 0;i<0x10;i++){
        //       stop();
        /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
      	SCB_SCR |= SCB_SCR_SLEEPDEEP_MASK;	
        GPIOE_PCOR = 0x04000000;    // clear Port E 26 indicating sleep
    	  /* WFI instruction will start entry into STOP mode */
    	  asm("WFI");
        // should never reach this code - recovery through reset.
        
        printf("MC_Exited VLLS3 Loop ERROR !!!! %#02X \r\n", (i)  );
        
        //if character has been received exit //
        if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
          break;
        } //if char received  */
    } //	for  
    printf("  MC_Exited VLLS3  \n");
    re_init_clk = 0;
   /*
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */
   
}
void en_int (int argc, char **argv)
{
    EnableInterrupts;
    printf("\r\n Interrupts Enabled \r\n");

}
void dis_int (int argc, char **argv)
{
    DisableInterrupts;
    printf("\r\n Interrupts Disabled \r\n");
}

//******************************************************************************

void
setshow_sysclk (int argc, char **argv)
{
    int result = 0;
    res_ptr = &result;
    printf("Placeholder for now...\n\n");
}
//******************************************************************************
