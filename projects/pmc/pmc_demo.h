/*
 * File:    PMC.h
 * Purpose: PMC functional validation header
 *
 * Notes:
 */

#ifndef _PMC_H_
#define _PMC_H_

/********************************************************************/
/*
 * Function Prototypes 
 */
void mainloop (void);
void cmd_rd (int, char **);
void cmd_rd_mcg (int argc, char **argv);
void cmd_wrt (int, char **);
void cmd_wrt_mcg (int argc, char **argv);
void set_en_rtc (int,char **); 
void set_en_lpt (int argc, char **argv);
void setshow_sysclk (int, char **);
void tst_stop_mode (int,char **); 
void tst_wait_mode (int,char **); 
void enter_vlpr_mode (int,char **);
void exit_vlpr_mode (int,char **);
void tst_vlps_mode (int,char **);
void tst_lls_mode (int,char **);
void tst_vlls1_mode (int,char **);
void tst_vlls2_mode (int,char **);
void tst_vlls3_mode (int,char **);
void cmd_atc (int argc, char **argv);
void en_int (int argc, char **argv); 
void dis_int (int argc, char **argv); 

extern void default_isr(void);
extern void llwu_isr(void);
extern void mcg_isr(void);
extern void porta_isr(void);
extern void portc_isr(void);
extern void porte_isr(void);
extern void pmc_lvd_isr(void);
extern void rtc_isr(void);
extern void lpt_isr(void);
extern void ftm2_isr(void); //isr for FTM2
extern void timer_setup(void);
extern void time_delay_ms(int);
extern void Port_Init(void);


#define USE_JTAG_MUXED_CHANNELS
#undef USE_JTAG_MUXED_CHANNELS



/********************************************************************/

#endif /* _PMC_H_ */
