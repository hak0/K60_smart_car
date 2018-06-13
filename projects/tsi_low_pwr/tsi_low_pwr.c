/*
 * File:		main.c
 * Purpose:		TSI Example
 *
 *                      Toggle the LED's on the tower board by pressing the TSI
 *                        touch pads
 *
 */

#include "common.h"
#include "TSI.h"
#include "misc.h"
#include "pmc.h"

void GPIO_Init(void);

/********************************************************************/
void main (void)
{
    RTC_Init();
    TSI_Init();
    GPIO_Init();
    TSI_SelfCalibration();

    START_SCANNING;
    ENABLE_EOS_INT;
    ENABLE_TSI;

    while(1)
    {
        /* WAIT mode */
        printf("\r\n Send any character to go into low power mode WAIT \r\n");
        LEDs_Off();
        in_char();
        printf("Entering low power WAIT mode \r\n");
        TSI_GoToLP();
        wait(); 
        printf("\r\n Woke up from Wait !!! \r\n");
        LEDs_On();

        /* STOP mode */
        printf("\r\n Send any character to go into low power mode STOP \r\n");
        LEDs_Off();
        in_char();
        printf("Entering low power STOP mode \r\n");
        enter_stop_mode(0); 
        mcg_pbe_2_pee();
        printf("\r\n Woke up from Stop - Getting better!!! \r\n");
        LEDs_On();

        /* VLPS mode */
        printf("\r\n Send any character to go into low power mode VLPS \r\n");
        LEDs_Off();
        in_char();
        enter_stop_mode(2); 
        mcg_pbe_2_pee();
        printf("\r\n Woke up from VLPS ! -  Cool!\r\n");
        LEDs_On();
    
        /* LLS mode */
        printf("\r\n Send any character to go into low power mode LLS \r\n");
        LEDs_Off();
        in_char();
        enter_stop_mode(3); 
        mcg_pbe_2_pee();
        printf("\r\n Woke up from LLS !!! much lower \r\n");
        LEDs_On();

        printf("\r\n Send any character to go into low power mode VLLS1 \r\n");
        in_char();
        enter_vlls1(); 
        mcg_pbe_2_pee();
        printf("\r\n Woke up from VLLS1 !!!  - Error !!! \r\n");
        LEDs_On();
        

    }

}
/********************************************************************/

