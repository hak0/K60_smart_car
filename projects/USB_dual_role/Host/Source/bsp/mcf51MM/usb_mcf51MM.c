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
* $FileName: usb_mcf51JM.c$
* $Version : 3.0.9.0$
* $Date    : Nov-21-2008$
*
* Comments:
*
*   This file contains board-specific initialization functions.
*
*END************************************************************************/
#include "usb_bsp.h"
#include "MCF51MM256.h"
#include "psptypes.h"
#include "usb_lock.h"
/* unsecure flash */
const unsigned char sec@0x040f = 0x00; 
/* checksum bypass */
const byte checksum_bypass@0x040a = 0x0; 

volatile uint_8 usb_int_disable = 0;
volatile uint_8 usb_int_en_copy = 0;

/***************************************************************************
 * Local Functions Prototypes 
 ***************************************************************************/
static void _bsp_mm256_clock_init_16MHz(void);
/***************************************************************************
 * Local Functions 
 ***************************************************************************/
 
 /*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _clock_init_12MHz
* Returned Value   : void
* Comments         :
*   
*
*END*----------------------------------------------------------------------*/

void usb_int_dis(void)
 {
  asm { move.w SR,D0; ori.l #0x0700,D0; move.w D0,SR;  }
  usb_int_disable++;
 }

void usb_int_en(void)
 {
  volatile uint_8  dummy;
  if(usb_int_disable) 
  {
  usb_int_disable--;
  }
  
  if(INT_ENB == 0)
    {
    if(usb_int_disable == 0) 
      {
       dummy = usb_int_en_copy;
       usb_int_en_copy = 0;
       INT_ENB = dummy;  
      }
    }
  if(usb_int_disable == 0)
    {
     asm { move.w SR,D0; andi.l #0xF8FF,D0; move.w D0,SR;  }
    }
 }
 
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _clock_init_12MHz
* Returned Value   : void
* Comments         :
*   
*
*END*----------------------------------------------------------------------*/
static void _bsp_mm256_clock_init_16MHz(void) 
{
         
  /* In order to use the USB we need to enter PEE mode and MCGOUT set to 48 MHz. 
     Out of reset MCG is in FEI mode. */
  
  //------FBE MODE------  
  /* Assume 16MHz external clock source connected. */
  // RANGE = 1; HGO = 1; ERCLKEN=1; EREFS = 1; BDIV = 000
  MCGC2 = 0x36;
  
  // DIV32 =1
  MCGC3 = 0x10;
  
  // CLKS = 10; RDIV = 100; IREFS = 0;
  MCGC1 = 0xA0; 


  // Wait for Reference Status bit to update
  while (MCGSC_IREFST)
  {
  }
  // Wait for clock status bits to update 
  while (MCGSC_CLKST != 0b10) 
  {
  }

//------PBE MODE------ 

  // PLLS =1; DIV32 = 1; VDIV = 1001 
  MCGC3 = 0x5c;
  
  // wait for PLL status bit to update
  while (!MCGSC_PLLST) 
  {
  }
  // Wait for LOCK bit to set 
  while (!MCGSC_LOCK) 
  {
  }
  
//------PEE MODE------   

  // CLKS = 00; RDIV = 100; IREFS = 0
  MCGC1 = 0x20;

// Wait for clock status bits to update 
  while (MCGSC_CLKST != 0b11) 
  {
  }
  }

/***************************************************************************
 * Global Functions 
 ***************************************************************************/
 
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_platform_init
* Returned Value   : 0 for success
* Comments         :
*    This function performs BSP-specific related to the MCF51MM platform
*
*END*----------------------------------------------------------------------*/

void _bsp_platform_init(void)
{
 
  SOPT1 = SOPT1_STOPE_MASK | SOPT1_WAITE_MASK | SOPT1_RSTPE_MASK | SOPT1_BKGDPE_MASK;   
  SOPT2 = 0;
  SOPT3 = SOPT3_SCI1PS_MASK;  /* route TX1,RX1 to PTD6,PTD7 */
  SPMSC1 = SPMSC1_LVDACK_MASK;
  SPMSC2 = 0;
  _bsp_mm256_clock_init_16MHz(); 
   
  
}  

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_usb_host_init
* Returned Value   : 0 for success
* Comments         :
*    This function performs BSP-specific initialization related to USB host
*
*END*----------------------------------------------------------------------*/
int_32 _bsp_usb_host_init(pointer param)
{
    SCGC2 |= SCGC2_USB_MASK;       
    SOPT2 &= ~SOPT2_USB_BIGEND_MASK;
    USBTRC0 |= (USBTRC0_USBPU_MASK | USBTRC0_USBVREN_MASK); 
        
    return 0;
}





