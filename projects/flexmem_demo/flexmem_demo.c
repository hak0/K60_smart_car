/*
 * File:		flexmem_demo.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "flexmem_demo.h"

/* Define globals for address of counters */
#define LONGWORD_COUNTER_ADDR 0x14000000
#define WORD_COUNTER_ADDR 0x14000004
#define BYTE_COUNTER_ADDR 0x14000006

/* Pull in global from sysinit.c for the peripheral clock frequency */
extern int periph_clk_khz;

/********************************************************************/
void main (void)
{
	char ch;
        uint32 timeout;
                 SCB_SHCSR|=SCB_SHCSR_BUSFAULTENA_MASK|SCB_SHCSR_MEMFAULTENA_MASK|SCB_SHCSR_USGFAULTENA_MASK;
  	printf("\nRunning FlexMem demo!!\n");

        /* Partition the memory to enable FlexMem mode */
        if ( partition_flash( EEPROM_16_16, DFLASH_SIZE_128) )
        {
            /* Device has been partitioned for the first time, so this
             * means the counters have not been initialized yet. We'll
             * zero them out now.
             */
            *((uint32 *)(LONGWORD_COUNTER_ADDR)) = 0x0;

            /* Wait for the command to complete */
            while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

            *((uint16 *)(WORD_COUNTER_ADDR)) = 0x0;

            /* Wait for the command to complete */
            while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

            *((uint8 *)(BYTE_COUNTER_ADDR)) = 0x0;

            /* Wait for the command to complete */
            while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));
        }

        /* Display the initial counter values */
        printf("\nlongword counter = 0x%08X", *(uint32 *)(LONGWORD_COUNTER_ADDR));
        printf("\nword counter = 0x%04X", *(uint16 *)(WORD_COUNTER_ADDR));
        printf("\nbyte counter = 0x%02X", *(uint8 *)(BYTE_COUNTER_ADDR));


        /* Initialize the PIT timer to generate an interrupt every 15s */

        /* Enable the clock to the PIT module */
        SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;

        /* Enable the PIT timer module */
        PIT_MCR &= ~PIT_MCR_MDIS_MASK;

        /* Calculate the timeout value to get a 15s interval */
        timeout = 15 * periph_clk_khz * 1000;
        PIT_LDVAL0 = timeout;

        /* Enable the timer and enable interrupts */
        PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK;

        /* Enable the PIT timer interrupt in the NVIC */
        enable_irq(68);


	while(1)
	{
		ch = in_char();
		out_char(ch);
	}
}
/********************************************************************/
/* Partition flash routine. This function can be used to setup
 * the flash for enhanced EEPROM operation. In order to guarantee
 * the eEE endurance the partition command should only be used one
 * time (re-partitioning in a different configuration will erase
 * wear-leveling data, so endurance can no longer be guaranteed).
 * This function will test to make sure the flash has not been
 * partitioned already.
 *
 * Parameters:
 * eeprom_size size of the two EEPROM data sets (A and B) defines in flexmem_demo.h
 * dlfash_size amount of dflash memory available after partitioning defines in flexmem_demo.h
 *
 * Returns:
 * 1  partitioning completed successfully
 * 0  partitioning not completed (device is already partitioned)
 */
int partition_flash(int eeprom_size, int dflash_size)
{
      /* Test to make sure the device is not already partitioned. If it
       * is already partitioned, then return with no action performed.
       */
      if ((SIM_FCFG1 & SIM_FCFG1_DEPART(0xF)) != 0x00000F00)
      {
          printf("\nDevice is already partitioned.\n");
          return 0;
      }

      /* Write the FCCOB registers */
      FTFL_FCCOB0 = FTFL_FCCOB0_CCOBn(0x80); // Selects the PGMPART command
      FTFL_FCCOB1 = 0x00;
      FTFL_FCCOB2 = 0x00;
      FTFL_FCCOB3 = 0x00;

      /* FCCOB4 is written with the code for the subsystem sizes (eeprom_size define) */
      FTFL_FCCOB4 = eeprom_size;

      /* FFCOB5 is written with the code for the Dflash size (dflash_size define) */
      FTFL_FCCOB5 = dflash_size;


      /* All required FCCOBx registers are written, so launch the command */
      FTFL_FSTAT = FTFL_FSTAT_CCIF_MASK;

      /* Wait for the command to complete */
      while(!(FTFL_FSTAT & FTFL_FSTAT_CCIF_MASK));

      return 1;
}
/********************************************************************/
/* PIT0 Interrupt Service Routine
 *
 * Updates the counters in EEPROM
 *
 */
void pit0_isr(void)
{
    uint32 temp;

    printf("\n\nPIT0 interrupt handler.");

    /* Clear the PIT timer flag */
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;

    /* Read the load value to restart the timer */
    temp = PIT_LDVAL0;

    /* Increment all three counters */

    /* Make sure the EEE is ready. If not wait for the command to complete */
    while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

    temp = *((uint32 *)(LONGWORD_COUNTER_ADDR));
    *((uint32 *)(LONGWORD_COUNTER_ADDR)) = (uint32) temp + 1;

    /* Make sure the EEE is ready. If not wait for the command to complete */
    while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

    temp = *((uint16 *)(WORD_COUNTER_ADDR));
    *((uint16 *)(WORD_COUNTER_ADDR)) = (uint16) temp + 1;

    /* Make sure the EEE is ready. If not wait for the command to complete */
    while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

    temp = *((uint8 *)(BYTE_COUNTER_ADDR));
    *((uint8 *)(BYTE_COUNTER_ADDR)) = (uint8) temp + 1;

    /* Make sure the EEE is ready. If not wait for the command to complete */
    while(!(FTFL_FCNFG & FTFL_FCNFG_EEERDY_MASK));

    /* Display the new counter values */
    printf("\nlongword counter = 0x%08X", *(uint32 *)(LONGWORD_COUNTER_ADDR));
    printf("\nword counter = 0x%04X", *(uint16 *)(WORD_COUNTER_ADDR));
    printf("\nbyte counter = 0x%02X", *(uint8 *)(BYTE_COUNTER_ADDR));

    return;
}
/********************************************************************/
