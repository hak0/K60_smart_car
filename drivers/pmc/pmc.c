/*
 * File:        pmc.c
 * Purpose:     Provides routines for entering low power modes.
 *
 * Notes:	Since the wakeup mechanism for low power modes
 *              will be application specific, these routines
 *              do not include code to setup interrupts to exit
 *              from the low power modes. The desired means of
 *              low power mode exit should be configured before
 *              calling any of these functions.
 *
 *              These routines do not include protection to
 *              prevent illegal state transitions in the mode
 *              controller, and all routines that write to the
 *              PMPROT register write a value to allow all
 *              possible low power modes (it is write once, so
 *              if only the currently requested mode is enabled
 *              a different mode couldn't be enabled later on).
 *              
 */

#include "common.h"
#include "pmc.h"
#include "mcg.h"
#include "uart.h"
extern int core_clk_khz;
extern int core_clk_mhz;
extern int periph_clk_khz;
int i;
       
/********************************************************************/
/* WAIT mode entry routine. Puts the processor into wait mode.
 * In this mode the core clock is disabled (no code executing), but 
 * bus clocks are enabled (peripheral modules are operational). 
 *
 * Mode transitions:
 * RUN -> WAIT
 * VLPR -> VLPW
 *
 * This function can be used to enter normal wait mode or VLPW
 * mode. If you are executing in normal run mode when calling this
 * function, then you will enter normal wait mode. If you are in VLPR
 * mode when calling this function, then you will enter VLPW mode instead.
 *
 * NOTE: Some modules include a programmable option to disable them in 
 * wait mode. If those modules are programmed to disable in wait mode, 
 * they will not be able to generate interrupts to wake up the core.
 *
 * WAIT mode is exited using any enabled interrupt or RESET, so no
 * exit_wait routine is needed. 
 *
 * If in VLPW mode, the statue of the MC_PMCTRL[LPWUI] bit determines if 
 * the processor exits to VLPR (LPWUI cleared) or normal run mode (LPWUI 
 * set). The enable_lpwui() and disable_lpwui()functions can be used to set
 * this bit to the desired option prior to calling enter_wait().
 * 
 * 
 * Parameters:
 * none
 */
void enter_wait(void)
{
    wait();
}
/********************************************************************/
/* STOP mode entry routine. Puts the processor into normal stop mode.
 * In this mode core, bus and peripheral clocks are disabled.
 *
 * Mode transitions:
 * RUN -> STOP
 * VLPR -> VLPS
 *
 * This function can be used to enter normal stop mode or VLPS
 * mode. If you are executing in normal run mode when calling this
 * function, then you will enter normal stop mode. If you are in VLPR
 * mode when calling this function, then you will enter VLPS mode instead.
 *
 * STOP mode is exited using any enabled interrupt or RESET, so no
 * exit_stop routine is needed.
 *
 * Parameters:
 * none
 */
void enter_stop(void)
{
    /* Set the LPLLSM field to 0b000 for normal STOP mode - Need to retain state of LPWUI bit 8 */
    MC_PMCTRL =  MC_PMCTRL_LPLLSM(0);           // set LPLLSM = 0b000
    stop();
}
/********************************************************************/
/* VLPR mode entry routine. Puts the processor into very low power
 * run mode. In this mode all clocks are enabled, but the core, bus,
 * and peripheral clocks are limited to 2MHz or less. The flash 
 * clock is limited to 1MHz or less. 
 *
 * Mode transitions:
 * RUN -> VLPR
 *
 * exit_vlpr() function or an interrupt with LPWUI set can be used 
 * to switch from VLPR back to RUN. The enable_lpwui() and disable_lpwui()
 * functions can be used to set LPWUI to the desired option prior to 
 * calling enter_vlpr().
 *
 * Parameters:
 * none
 */
void enter_vlpr(char lpwui_value)
{
     MC_PMPROT = MC_PMPROT_AVLP_MASK;  // write oneif not all set make sure all enabled
                                       //this write-once bit allows the MCU to enter the
                                       //very low power modes: VLPR, VLPW, and VLPS.
    
   /* Reduce system clock to < 2MHz */
    printf("\n\n\n To communicate in VLPR - Auto-Trim must have been done, Change to 19200 baud after key hit !!!! \n\r");
    printf("\r\n Send any character to go into lower divide mode \r\n");
    in_char();

    mcg_pee_2_blpi();
    SIM_CLKDIV1  =  0x03330000;    //(SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(3) \
                                       // SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3));
                                       //core = 1 bus = /1 flexbus = /1 flash clk =/4
 
    //change the UART serial baud  rate to 19200 baud
    core_clk_khz = 1786;
    periph_clk_khz = core_clk_khz / (((SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> 24)+ 1);
    if ((TERM_PORT == UART0_BASE_PTR) | (TERM_PORT == UART1_BASE_PTR))
        uart_init (TERM_PORT, core_clk_khz, TERMINAL_BAUD);
    else
        uart_init (TERM_PORT, periph_clk_khz, TERMINAL_BAUD);
    // ~1.78MHz

    printf("\r\n Send any character to go into low power mode \r\n");
    in_char();

     /* Set the RUNM field to 0b10 for VLPR mode - Need to retain state of LPWUI bit 8 */
    /* Set the LPLLSM field to 0b010 for VLPS mode - Need to set state of LPWUI bit 8 */
    if(lpwui_value){
       MC_PMCTRL = (MC_PMCTRL_LPWUI_MASK       // set LPWUI
                    | MC_PMCTRL_RUNM(2));       // set RUNM = 0b10     
   
     } else {
       MC_PMCTRL = (!MC_PMCTRL_LPWUI_MASK            // clear LPWUI
                   | MC_PMCTRL_RUNM(2));             // set RUNM = 0b10 
     }       
        
    
      /* Wait for VLPS regulator mode to be confirmed */

     while((PMC_REGSC & PMC_REGSC_VLPRS_MASK)==0);    // 0 MCU is not in VLPR mode
                                                   // 1 MCU is in VLPR mode
     for (i= 0;i<0xffff;i++){
 //          printf(" Now in VLPR at 19200 baud  \r\n");
           printf("U");
              if ((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK)) {
                 break;
              }//if char received  */
     }//for

}
/********************************************************************/
/* VLPR mode exit routine. Puts the processor into normal run mode
 * from VLPR mode. You can transition from VLPR to normal run using
 * this function or an interrupt with LPWUI set. The enable_lpwui() 
 * and disable_lpwui() functions can be used to set LPWUI to the 
 * desired option prior to calling enter_vlpr().
 *
 * Mode transitions:
 * VLPR -> RUN
 *
 * Parameters:
 * none
 */
void exit_vlpr(void)
{
    /* Clear RUNM */
    MC_PMCTRL &= ~(MC_PMCTRL_RUNM(0x3));
                   
    /* Wait for normal RUN regulation mode to be confirmed */                   
    while (PMC_REGSC & PMC_REGSC_VLPRS_MASK); // 0 MCU is not in VLPR mode
                                              // 1 MCU is in VLPR mode
    while(!(PMC_REGSC & PMC_REGSC_REGONS_MASK)); 
    
    /* Transition MCG back to the PLL enabled state */
    mcg_blpi_2_pee();
    //sim_clkdivided back to default
    SIM_CLKDIV1 = 0x00010000;   //(SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(3) \
                                // SIM_CLKDIV1_OUTDIV3(3) | SIM_CLKDIV1_OUTDIV4(3));
                                //core = /1 bus = /1 =flex bus /2 flash clk /2
    core_clk_khz = core_clk_mhz * 1000;
    periph_clk_khz = core_clk_khz / (((SIM_CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> 24)+ 1);
    if ((TERM_PORT == UART3_BASE_PTR) | (TERM_PORT == UART1_BASE_PTR))
         uart_init (TERM_PORT, core_clk_khz, TERMINAL_BAUD);
    else
         uart_init (TERM_PORT, periph_clk_khz, TERMINAL_BAUD);
    
    printf("               Now in Run Mode at 115200 baud \r");

}
/********************************************************************/
/* VLPS mode entry routine. Puts the processor into VLPS mode directly
 * from normal run mode. 
 *
 * Mode transitions:
 * RUN -> VLPS
 *
 * If VLPS mode is entered directly from normal RUN mode, then the 
 * LPWUI bit is forced to 1 by hardware. This means that when an
 * interrupt occurs you will exit to normal run mode instead of VLPR.
 *
 * If however VLPS mode is entered from VLPR the state of the LPWUI bit
 * determines the state the MCU will return to upon exit from VLPS.If LPWUI is set
 * and an interrupt occurs you will exit to normal run mode instead of VLPR. 
 * If LPWUI is clear and an interrupt occurs you will exit to VLPR.
 *
 * Parameters:  value of LPWUI
 * none
 */
void enter_vlps(char lpwui_value)
{
    /* Write to PMPROT to allow VLPS power modes */
    MC_PMPROT = MC_PMPROT_AVLP_MASK;   // write oneif not all set make sure all enabled
                                       //this write-once bit allows the MCU to enter the
                                       //very low power modes: VLPR, VLPW, and VLPS.
 
    /* Reduce system clock to < 2MHz */
 //   mcg_pee_2_blpi();    // you don't need to chage to lower frequency for VLPS
       
    /* Set the LPLLSM field to 0b010 for VLPS mode - Need to set state of LPWUI bit 8 */
    if(lpwui_value){
       MC_PMCTRL = (MC_PMCTRL_LPWUI_MASK            // set LPWUI
                  | MC_PMCTRL_LPLLSM(2));           // set LPLLSM = 0b10
     } else {
       MC_PMCTRL = (!MC_PMCTRL_LPWUI_MASK           // set LPWUI
                  | MC_PMCTRL_LPLLSM(2));           // set LPLLSM = 0b10
     }       
    /* Now execute the stop instruction to go into VLPS */
    stop();
}
/********************************************************************/
/* LLS mode entry routine. Puts the processor into LLS mode from
 * normal run mode or VLPR. 
 *
 * Mode transitions:
 * RUN -> LLS
 * VLPR -> LLS
 *
 * NOTE: LLS mode will always exit to RUN mode even if you were 
 * in VLPR mode before entering LLS.
 *
 * Wakeup from LLS mode is controlled by the LLWU module. Most
 * modules cannot issue a wakeup interrupt in LLS mode, so make
 * sure to setup the desired wakeup sources in the LLWU before 
 * calling this function.
 *
 * Parameters:
 * none
 */
void enter_lls(void)
{
    /* Write to PMPROT to allow LLS power modes */
    MC_PMPROT = MC_PMPROT_ALLS_MASK;   //this write-once bit allows the MCU to enter the
                                       //LLS low power mode

        
    /* Set the LPLLSM field to 0b011 for LLS mode  */
    MC_PMCTRL  =  MC_PMCTRL_LPLLSM(3);           // set LPLLSM = 0b11
        
    /* Now execute the stop instruction to go into LLS */
    stop();
}
/********************************************************************/
/* VLLS3 mode entry routine. Puts the processor into VLLS3 mode from
 * normal run mode or VLPR. 
 *
 * Mode transitions:
 * RUN -> VLLS3
 * VLPR -> VLLS3
 *
 * NOTE: VLLSx modes will always exit to RUN mode even if you were 
 * in VLPR mode before entering VLLSx.
 *
 * Wakeup from VLLSx mode is controlled by the LLWU module. Most
 * modules cannot issue a wakeup interrupt in VLLSx mode, so make
 * sure to setup the desired wakeup sources in the LLWU before 
 * calling this function.
 *
 * Parameters:
 * none  
 */
void enter_vlls3(void)
{
    /* Write to PMPROT to allow VLLS3 power modes */
    MC_PMPROT = MC_PMPROT_AVLLS3_MASK;
        
    /* Set the LPLLSM field to 0b101 for VLLS3 mode  */
    MC_PMCTRL =  MC_PMCTRL_LPLLSM(5);           // set LPLLSM = 0b101
    disable_ports();    
    /* Now execute the stop instruction to go into VLLS3 */
    stop();
}
/********************************************************************/
/* VLLS2 mode entry routine. Puts the processor into VLLS2 mode from
 * normal run mode or VLPR. 
 *
 * Mode transitions:
 * RUN -> VLLS2
 * VLPR -> VLLS2
 *
 * NOTE: VLLSx modes will always exit to RUN mode even if you were 
 * in VLPR mode before entering VLLSx.
 *
 * Wakeup from VLLSx mode is controlled by the LLWU module. Most
 * modules cannot issue a wakeup interrupt in VLLSx mode, so make
 * sure to setup the desired wakeup sources in the LLWU before 
 * calling this function.
 *
 * Parameters:
 * none  
 */
void enter_vlls2(void)
{
    /* Write to PMPROT to allow VLLS2 power modes */
    MC_PMPROT = MC_PMPROT_AVLLS2_MASK;
        
    /* Set the LPLLSM field to 0b110 for VLLS2 mode */
    MC_PMCTRL =  MC_PMCTRL_LPLLSM(6);           // set LPLLSM = 0b110
    disable_ports();    
        
    /* Now execute the stop instruction to go into VLLS2 */
    stop();
}
/********************************************************************/
/* VLLS1 mode entry routine. Puts the processor into VLLS1 mode from
 * normal run mode or VLPR. 
 *
 * Mode transitions:
 * RUN -> VLLS1
 * VLPR -> VLLS1
 *
 * NOTE: VLLSx modes will always exit to RUN mode even if you were 
 * in VLPR mode before entering VLLSx.
 *
 * Wakeup from VLLSx mode is controlled by the LLWU module. Most
 * modules cannot issue a wakeup interrupt in VLLSx mode, so make
 * sure to setup the desired wakeup sources in the LLWU before 
 * calling this function.
 *
 * Parameters:
 * none  
 */
void enter_vlls1(void)
{
    /* Write to PMPROT to allow all possible power modes */
    MC_PMPROT = MC_PMPROT_AVLLS1_MASK;
        
    /* Set the LPLLSM field to 0b111 for VLLS1 mode  */
    MC_PMCTRL =  MC_PMCTRL_LPLLSM(7);           // set LPLLSM = 0b111
    disable_ports();    
        
    /* Now execute the stop instruction to go into VLLS1 */
    stop();
}
/********************************************************************/
/* Enable low power wake up on interrupt. This function can be used
 * to set the LPWUI bit. When this bit is set VLPx modes will exit
 * to normal run mode. When this bit is cleared VLPx modes will exit
 * to VLPR mode.
 *
 * The disable_lpwui() function can be used to clear the LPWUI bit.
 *
 * Parameters:
 * none
 */

   

void enable_lpwui(void)
{
    MC_PMCTRL |= MC_PMCTRL_LPWUI_MASK;
}
/********************************************************************/
/* Disable low power wake up on interrupt. This function can be used
 * to clear the LPWUI bit. When this bit is set VLPx modes will exit
 * to normal run mode. When this bit is cleared VLPx modes will exit
 * to VLPR mode.
 *
 * The enable_lpwui() function can be used to set the LPWUI bit.
 *
 * Parameters:
 * none
 */
void disable_lpwui(void)
{
    MC_PMCTRL &= ~MC_PMCTRL_LPWUI_MASK;
}
/********************************************************************/
   /* entry mode variable is set to one of the valid settings following
    000 Normal stop
    001 Reserved
    010 Very low power stop (VLPS)
    011 Low leakage stop (LLS)
    100 Reserved
    101 Very low leakage stop 3 (VLLS3)
    110 Very low leakage stop 2 (VLLS2)
    111 Very low leakage stop 1 (VLLS1)
   */
/********************************************************************/
void enter_stop_mode(char mode)
{
    printf("\n-------------------------------------\n");
    printf("  MC_Enter Low Power Modes Utility    \n");
    printf("--------------------------------------\n");
    
    MC_PMPROT = 0x37;               //This value enables all possible power modes.
 
    MC_PMCTRL = MC_PMCTRL_LPWUI_MASK | MC_PMCTRL_LPLLSM(mode) ;  
    stop();         
}

void     disable_ports(void){
#if (defined(TWR_K60N512))
   PORTE_PCR26 = PORT_PCR_IRQC(00);;
   PORTA_PCR19 = PORT_PCR_IRQC(00);
   #endif
  
#if (defined(TWR_K40X256))
   //  SW6 and SW7
     PORTC_PCR5  = PORT_PCR_IRQC(00);
     PORTC_PCR13  = PORT_PCR_IRQC(00);
#endif
     
   // disable trace clock output
     PORTA_PCR6 = ( PORT_PCR_MUX(0x0));
   // disable JTAG i/o PTA0-3 5
     PORTA_PCR0 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR1 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR2 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR3 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR5 = ( PORT_PCR_MUX(0x0));

   // disable fb_clk output
     PORTC_PCR3 = ( PORT_PCR_MUX(0x0));
 	/* Disable the pins for the selected UART */
     if (TERM_PORT == UART0_BASE_PTR)
         {
            /* Disable the UART0_TXD function on PTD6 */
            PORTD_PCR6 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

            /* Disable the UART0_RXD function on PTD7 */
            PORTD_PCR7 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
         }

     if (TERM_PORT == UART1_BASE_PTR)
  	 {
                 /* Disable the UART1_TXD function on PTC4 */
  		PORTC_PCR4 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		/* Enable the UART1_RXD function on PTC3 */
  		PORTC_PCR3 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}

    if (TERM_PORT == UART2_BASE_PTR)
  	{
                 /* Disable the UART2_TXD function on PTD3 */
  		PORTD_PCR3 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		/* Disable the UART2_RXD function on PTD2 */
  		PORTD_PCR2 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}

    if (TERM_PORT == UART3_BASE_PTR)
  	{
                 /* Disable the UART3_TXD function on PTC17 */
  		PORTC_PCR17 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		/* Enable the UART3_RXD function on PTC16 */
  		PORTC_PCR16 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
    if (TERM_PORT == UART4_BASE_PTR)
  	{
                 /* Disable the UART3_TXD function on PTC17 */
  		PORTE_PCR24 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		/* Disable the UART3_RXD function on PTC16 */
  		PORTE_PCR25 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
    if (TERM_PORT == UART5_BASE_PTR)
  	{
                 /* Disable the UART3_TXD function on PTC17 */
  		PORTE_PCR8 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		/* Disable the UART3_RXD function on PTC16 */
  		PORTE_PCR9 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
  // disable clock gating
  SIM_SCGC1 = 0;
  SIM_SCGC2 = 0;
  SIM_SCGC3 = 0;
  SIM_SCGC4 = 0;
  SIM_SCGC5 = 0;
  SIM_SCGC6 = 0;
  SIM_SCGC7 = 0;

}

//AJJ
void     disable_ports_partial(void){
/*
#if (defined(TWR_K60N512))
   PORTE_PCR26 = PORT_PCR_IRQC(00);;
   PORTA_PCR19 = PORT_PCR_IRQC(00);
   #endif
  
#if (defined(TWR_K40X256))
   //  SW6 and SW7
     PORTC_PCR5  = PORT_PCR_IRQC(00);
     PORTC_PCR13  = PORT_PCR_IRQC(00);
#endif
*/    
   // disable trace clock output
     PORTA_PCR6 = ( PORT_PCR_MUX(0x0));
   // disable JTAG i/o PTA0-3 5
     PORTA_PCR0 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR1 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR2 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR3 = ( PORT_PCR_MUX(0x0));
     PORTA_PCR5 = ( PORT_PCR_MUX(0x0));
/*
   // disable fb_clk output
     PORTC_PCR3 = ( PORT_PCR_MUX(0x0));
 	// Disable the pins for the selected UART 
     if (TERM_PORT == UART0_BASE_PTR)
         {
            // Disable the UART0_TXD function on PTD6 
            PORTD_PCR6 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

            // Disable the UART0_RXD function on PTD7 
            PORTD_PCR7 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
         }

     if (TERM_PORT == UART1_BASE_PTR)
  	 {
                 // Disable the UART1_TXD function on PTC4 
  		PORTC_PCR4 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		// Enable the UART1_RXD function on PTC3 
  		PORTC_PCR3 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}

    if (TERM_PORT == UART2_BASE_PTR)
  	{
                 // Disable the UART2_TXD function on PTD3 
  		PORTD_PCR3 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		// Disable the UART2_RXD function on PTD2 
  		PORTD_PCR2 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}

    if (TERM_PORT == UART3_BASE_PTR)
  	{
                 // Disable the UART3_TXD function on PTC17 
  		PORTC_PCR17 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		// Enable the UART3_RXD function on PTC16 
  		PORTC_PCR16 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
    if (TERM_PORT == UART4_BASE_PTR)
  	{
                 // Disable the UART3_TXD function on PTC17 
  		PORTE_PCR24 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		// Disable the UART3_RXD function on PTC16 
  		PORTE_PCR25 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
    if (TERM_PORT == UART5_BASE_PTR)
  	{
                 // Disable the UART3_TXD function on PTC17 
  		PORTE_PCR8 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin

  		// Disable the UART3_RXD function on PTC16 
  		PORTE_PCR9 = PORT_PCR_MUX(0x0); // UART is alt3 function for this pin
  	}
*/

     // disable clock gating
  //SIM_SCGC1 = 0;  //uart 4 & 5
  //SIM_SCGC2 = 0;  //DAC 0 & 1, ENET
//  SIM_SCGC3 = 0;  //ADC1(27), FTM2(24), SDHC(17), SPI2(12), FLEXCAN1(4), RNGB(0)

//  printf("SIM_SCGC4 is %08x \n\r\n", SIM_SCGC4);
//  printf("SIM_SCGC4 is %08x \n\r\n", SIM_SCGC4);

  SIM_SCGC4 = SIM_SCGC4 & 0x70102030;  //LLWU(28), VREF(20), CMP(19), USBOTG(18), UART3 - 0(13-10), 
                  //I2C1(7), I2C0(6), CMT(2), EWM(1)   VREF defaults to 1
//  printf("SIM_SCGC4 is %08x \n\r\n", SIM_SCGC4);
//  while(!(UART3_S1&UART_S1_TC_MASK));  //make sure uart buffer is empty before exiting


  //SIM_SCGC5 = 0;  //PortE - A(13-9), TSI(5), REGFILE(1), LPTIMER(0)
  //SIM_SCGC6 = 0;  //RTC(29), ADC0(27), FTM1(25), FTM0(24), PIT(23), PDB(22), USBDCD(21), CRC(18), 
                  //I2S(15), SPI1(13), DSPI0(12), FLEXCAN0(4), DMAMUX(1), FTFL(0)
  SIM_SCGC7 = 0x00000000;  //MPU(2), DMA(1), FLEXBUS(0)


}

void outSRS(void){                         //[outSRS]
  if (MC_SRSH & MC_SRSH_SW_MASK)
		printf("[outSRS]Software Reset\n");
	if (MC_SRSH & MC_SRSH_LOCKUP_MASK)
		printf("[outSRS]Core Lockup Event Reset\n");
	if (MC_SRSH & MC_SRSH_JTAG_MASK)
		printf("[outSRS]JTAG Reset\n");
	
	if (MC_SRSL & MC_SRSL_POR_MASK)
		printf("[outSRS]Power-on Reset\n");
	if (MC_SRSL & MC_SRSL_PIN_MASK){
	    // find out which mode we are wakeing up from
          printf("External Pin Reset\n");
	}
	if (MC_SRSL & MC_SRSL_WAKEUP_MASK){
	     printf("[outSRS]Pin Reset wakeup from low power modes\n");
       //The state of PMCTRL[LPLLSM] prior to clearing due to update 
       // of PMPROT indicates which power mode was exited and should be 
       // used by initialization software for proper power mode recovery.
          if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 0)
             printf("[outSRS]Pin Reset wakeup from Normal Stop\n");
	        if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 2)
		         printf("[outSRS]Pin Reset wakeup from Very Low Power Stop(VLPS)\n");
		      if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 3)
		         printf("[outSRS]Pin Reset wakeup from Low Leakage Stop (LLS)\n");
		      if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 5 )
		         printf("[outSRS]Pin Reset wakeup from Very low leakage stop (VLLS3)\n");
		      if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 6)
		         printf("[outSRS]Pin Reset wakeup from Very low leakage stop (VLLS2)\n");
		      if ((MC_PMCTRL & MC_PMCTRL_LPLLSM_MASK) == 7)		       
		         printf("Pin Reset wakeup from Very low leakage stop 1(VLLS1)\n");
          printf("[outSRS]MC PMPROT= %#02X \r\n",   (MC_PMPROT)  );
          printf("[outSRS]MC PMCTRL= %#02X \r\n\n", (MC_PMCTRL) ) ;
	}
	if (MC_SRSL & MC_SRSL_COP_MASK)
	 	printf("[outSRS]Watchdog(COP) Reset\n");
  if (MC_SRSL & MC_SRSL_LOC_MASK)
		printf("[outSRS]Loss of Clock Reset\n");
	if (MC_SRSL & MC_SRSL_LVD_MASK)
	  printf("[outSRS]Low-voltage Detect Reset\n");
  if (MC_SRSL & MC_SRSL_WAKEUP_MASK)
		printf("[outSRS]LLWU Reset\n");	
}




