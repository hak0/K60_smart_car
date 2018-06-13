/**HEADER********************************************************************
* 
* Copyright (c) 2010 Freescale Semiconductor;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* $FileName:
* $Version :
* $Date    :
*
* Comments:
*
*
*END************************************************************************/
#include "sci.h"
#include "psptypes.h"
#include "fio.h"
#include "usb.h"
#include "io.h"
//#include "derivative.h"
#include "rtc.h"
#include "host_common.h"

static int_16 _bsp_timer_sw_prescaller;
static int_16 _bsp_timer_sw_prescaller_cnt;

void TimerInit(void);
static void EnableTimerInterrupt(void);
static void DisableTimerInterrupt(void);


//void interrupt VectorNumber_Vrtc_wdg Timer_ISR(void);
void  Timer_ISR(void);

static TICK_STRUCT time_ticks;

uint_32 delay_count;
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : TimerInit
* Returned Value   :
* Comments         : Initialize timer module
*    
*
*END*----------------------------------------------------------------------*/
void TimerInit(void) 
{ 	
    extern uint32 __VECTOR_RAM[];           //Get vector table that was copied to RAM

    /* Enable LPT Module Clock */
    SIM_SCGC5|=SIM_SCGC5_LPTIMER_MASK;

    /* Configure LPT */

    /* Enable LPT Interrupt in NVIC*/
    __VECTOR_RAM[101]=(uint_32)Timer_ISR;      //replace ISR
    NVICICPR2|=(1<<21);   //Clear any pending interrupts on LPT
    NVICISER2|=(1<<21);   //Enable interrupts from LPT module  
  
  /*
    RTCSC_RTIF = 0x01; // Clear previous RTC Interrupt 
	RTCMOD = 0x0B;	// 1 ms Interrupt Generation 
	// Start RTC by Reseting Counter to 0 
	RTCSC = 0x08;	// Prescaler = 1000 Clock = 12MHz (external clock ) 
	//RTCSC = 0xA8;	// Prescaler = 1000 Clock = 12MHz (external clock )       
*/


  EnableTimerInterrupt();
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : EnableTimerInterrupt 
* Returned Value   :
* Comments         : Enable timer interrupt
*    
*
*END*----------------------------------------------------------------------*/
static void EnableTimerInterrupt(void)
{
	/* Enable Timer Interrupt */
	//RTCSC_RTIE = 0x01;

        return;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : DisableTimerInterrupt 
* Returned Value   :
* Comments         : Disable timer interrupt.
*    
*
*END*----------------------------------------------------------------------*/
static void DisableTimerInterrupt(void)
{
	/* Disable Timer Interrupt */
	//RTCSC_RTIE = 0x00;

	return;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : Timer_ISR 
* Returned Value   :
* Comments         : Timer interrupt service routine
*    
*
*END*----------------------------------------------------------------------*/
void  Timer_ISR(void)
{
/*	uint_8 index;
    if(RTCSC & RTCSC_RTIF_MASK)
    {
		// Clear RTC Interrupt 
		RTCSC_RTIF = 1; // Write 1 to Clear 
    if(0 <= delay_count)
      delay_count--;
		
		
	  }
*/



    if(0 <= delay_count)
      delay_count--;
    printf("LPT\r\n");

}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : time_delay 
* Returned Value   :
* Comments         : Wait until interrupt of timer occur
*    
*
*END*----------------------------------------------------------------------*/
void time_delay(uint_32 delay) 
{
    uint_32 u32Delay;
    uint_32 u32Delay2;
    uint_32 u32Delay3;
  
  /*
  delay_count = delay;
  //RTCMOD = 0x0B; // Reset RTCCNT 
  while(1){    
    if(0 == delay_count){
      break;
    }
  }
*/
  
    for(u32Delay=0;u32Delay<delay;u32Delay++);
        for(u32Delay2=0;u32Delay2<0xFFFF;u32Delay2++);
            //for(u32Delay3=0;u32Delay3<0xFFFFFFFF;u32Delay3++);  
      
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : get_hw_ticks 
* Returned Value   : hardware tick per prescaler.
* Comments         :
*    
*
*END*----------------------------------------------------------------------*/
uint_32 get_hw_ticks(void) 
{
/*
  uint_32 ticks;   
  
  ticks = (uint_16)RTCCNT;   

  if ((RTCSC & RTCSC_RTIF_MASK) != 0) 
  {
    ticks = (uint_16)RTCMOD + (uint_16)RTCCNT;
  } 
  
  return ticks;
 */
  return(100);
}


