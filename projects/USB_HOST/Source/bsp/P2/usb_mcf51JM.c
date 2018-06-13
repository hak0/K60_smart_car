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
#include "MCF51Z128.h"


/***************************************************************************
 * Local Functions Prototypes 
 ***************************************************************************/
static void _bsp_jm128_clock_init_12MHz(void);
static void _bsp_jm128_clock_init_4MHz(void) ;

/***************************************************************************
 * Local Functions 
 ***************************************************************************/

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _clock_init_4MHz
* Returned Value   : void
* Comments         :
*   
*
*END*----------------------------------------------------------------------*/
static void _bsp_jm128_clock_init_4MHz(void) 
{
    PTEPF2 = 0x05;             //SCI TX and RX control pad
    PTGDS=0xFF;
    PTGSRE=0x00;
    OUTDIV12=0x01;
    OUTDIV34=0x00;

    MCGC2 = 0x2C;
    MCGC1 = 0x98;
    while (!MCGS_OSCINIT){};        // wait for oscillator to initialize 
    while (MCGS_IREFST){};          /* wait for Reference Status bit to update */
    while (MCGS_CLKST != 0b10){};   // Wait for clock status bits to update
    MCGC5 = 0x45;
    MCGC6 = 0x40;
    while (!MCGS_PLLST){};          /* wait for PLL status bit to update */
    while (!MCGS_LOCK){};           /* Wait for LOCK bit to set */
    MCGC1 = 0x18;
    while (MCGS_CLKST != 0b11){};           
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _clock_init_12MHz
* Returned Value   : void
* Comments         :
*   
*
*END*----------------------------------------------------------------------*/
static void _bsp_jm128_clock_init_12MHz(void) 
{
    return; 
}

/***************************************************************************
 * Global Functions 
 ***************************************************************************/
 
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_platform_init
* Returned Value   : 0 for success
* Comments         :
*    This function performs BSP-specific related to the MCF51JM platform
*
*END*----------------------------------------------------------------------*/

void _bsp_platform_init(void)
{
/*
  SOPT1 = SOPT1_STOPE_MASK | SOPT1_WAITE_MASK;   
  SOPT2 = 0;
  SPMSC1 = SPMSC1_LVWACK_MASK;
  SPMSC2 = 0;
*/
  //_bsp_jm128_clock_init_12MHz();  

   PTHDD_PTHDD4=1;
   PTHD_PTHD4=0; 
  _bsp_jm128_clock_init_4MHz();  
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





