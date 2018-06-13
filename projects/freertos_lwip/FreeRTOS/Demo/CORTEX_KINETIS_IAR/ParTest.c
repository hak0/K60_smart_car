/*
	FreeRTOS.org V5.0.3 - Copyright (C) 2003-2008 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

    ***************************************************************************
    ***************************************************************************
    *                                                                         *
    * SAVE TIME AND MONEY!  We can port FreeRTOS.org to your own hardware,    *
    * and even write all or part of your application on your behalf.          *
    * See http://www.OpenRTOS.com for details of the services we provide to   *
    * expedite your project.                                                  *
    *                                                                         *
    ***************************************************************************
    ***************************************************************************

	Please ensure to read the configuration and relevant port sections of the
	online documentation.

	http://www.FreeRTOS.org - Documentation, latest information, license and
	contact details.

	http://www.SafeRTOS.com - A version that is certified for use in safety
	critical systems.

	http://www.OpenRTOS.com - Commercial support, development, porting,
	licensing and training services.
*/


#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

#include "common.h"

#define partstNUM_LEDs	4

#define GPIO_PIN_MASK            0x1Fu
#define GPIO_PIN(x)              (((1)<<(x & GPIO_PIN_MASK)))

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

void vParTestInitialise( void )
{
  /*
    PTA10 - Blue (D17)
    PTA29 - Green (D14)
    PTA28 - Yellow (D15)
    PTA11 - Orange (D16)
  */  
  
  /*LEDs are off and ready to be off*/
  //Set PTA10, PTA11, PTA28, and PTA29 to GPIO
  PORTA_PCR10=(0|PORT_PCR_MUX(1));
  PORTA_PCR11=(0|PORT_PCR_MUX(1));
  PORTA_PCR28=(0|PORT_PCR_MUX(1));
  PORTA_PCR29=(0|PORT_PCR_MUX(1));

  //Set PTA10, PTA11, PTA28, PTA29 as an output
  GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
unsigned portCHAR ucLED = ( unsigned portCHAR ) ( ( unsigned portBASE_TYPE ) 2 << uxLED );

	if( uxLED < partstNUM_LEDs )
	{
		if( xValue == pdFALSE )
		{
			portENTER_CRITICAL();
			{
                                GPIOA_PDDR&=~GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
			}
			portEXIT_CRITICAL();
		}
		else
		{
			portENTER_CRITICAL();
			{
                                GPIOA_PDDR|=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
			}
			portEXIT_CRITICAL();			
		}
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
unsigned portCHAR ucLED = ( unsigned portCHAR ) ( ( unsigned portBASE_TYPE ) 2 << uxLED );

	if( uxLED < partstNUM_LEDs )
	{
		portENTER_CRITICAL();
		{
                                GPIOA_PDDR^=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );
		}
		portEXIT_CRITICAL();
	}
}
/*-----------------------------------------------------------*/

unsigned portBASE_TYPE uxParTestGetLED( unsigned portBASE_TYPE uxLED )
{
unsigned portBASE_TYPE uxReturn = pdFALSE;

	if( uxLED < partstNUM_LEDs )
	{
		portENTER_CRITICAL();
		{            
			uxReturn = !(GPIOA_PDOR & GPIO_PDOR_PDO(GPIO_PIN(10)));
		}
		portEXIT_CRITICAL();
	}
        
        return uxReturn;
}


