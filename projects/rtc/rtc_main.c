/*
 * File:		rtc.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "rtc.h"

/********************************************************************/
void main (void)
{
	char ch;
                
  	printf("\nRunning RTC example!!\n");
        
        printf("This example generates an interrupt every second!\n");
        
        /* Parameters:
         *  seconds         Start value of seconds register
         *  alarm           Time in seconds of first alarm. Set to 0xFFFFFFFF to effectively disable alarm
         *  c_interval      Interval at which to apply time compensation can range from 1 second (0x0) to 256 (0xFF)
         *  c_value         Compensation value ranges from -127 32kHz cycles to +128 32 kHz cycles
         *                  80h Time prescaler register overflows every 32896 clock cycles.
         *                  FFh Time prescaler register overflows every 32769 clock cycles.
         *                  00h Time prescaler register overflows every 32768 clock cycles.
         *                  01h Time prescaler register overflows every 32767 clock cycles.
         *                  7Fh Time prescaler register overflows every 32641 clock cycles.
         *  interrupt       TRUE or FALSE
         */
        rtc_init(0, 0, 0, 0, TRUE);

	while(1)
	{
		ch = in_char();
		out_char(ch);
	} 
}
/********************************************************************/
