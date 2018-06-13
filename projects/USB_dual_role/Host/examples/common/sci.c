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
*
* Comments:  This file includes operation of serial communication interface.
*
*
*END************************************************************************/
#include "sci.h"
#include "psptypes.h"
#include "fio.h"
#include "usb.h"
#include "usb_bsp.h"
#include "io.h"
#include "derivative.h"
#include <Uart.h>

char   buff[200];
uint_32 buff_index;

void interrupt VectorNumber_Vsci2 SCI2RX_ISR(void);


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : sci1_init
* Returned Value   :
* Comments         : This function initilizes the SCI 1 baud rate.
*    
*
*END*----------------------------------------------------------------------*/
void sci1_init(void) 
{
 /* Configure SCI baud rate = 20K @16M */
  SCI1BDH = 0; 
  SCI1BDL = 0x0D;
  SCI1C1  = 0;
  SCI1C2  = 0x0C;
  SCI1S2  = 0x04;
  
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : TERMIO_PutChar
* Returned Value   :
* Comments         :
*                     This function sends a char via SCI.
*
*END*----------------------------------------------------------------------*/
void TERMIO_PutChar(char send) 
{
  char dummy;
  
  while(!SCI1S1_TDRE);
  dummy = SCI1S1;
  SCI1D  = send;    
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : TERMIO_GetChar
* Returned Value   : the char get via SCI
* Comments         :
*                     This function gets a char via SCI.
*
*END*----------------------------------------------------------------------*/
char TERMIO_GetChar(void) 
{
  char dummy;
  
  while(!SCI1S1_RDRF);
  dummy = SCI1S1;
  return SCI1D;  
}



/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : sci2_init
* Returned Value   :
* Comments         : This function initializes the SCI2 interrupt.
*    
*
*END*----------------------------------------------------------------------*/
void sci2_init(void) 
{
 /* Configure SCI baud rate = 20K @16M */
  SCI2BDH = 0; 
  SCI2BDL = 0x0D;
  SCI2C1  = 0;
  SCI2C2  = 0x0C;
  SCI2S2  = 0x04;
  SCI2C2_RE = 0x1;
  SCI2C2_RIE = 0x1;
  //SCI2C2_TIE = 0x1;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : SCI2RX_ISR
* Returned Value   :
* Comments         : Service interrupt in SCI2 hardware.
*    
*
*END*----------------------------------------------------------------------*/
void interrupt VectorNumber_Vsci2 SCI2RX_ISR(void) 
{  
  char tmp, dummy;
  tmp = SCI2D;
  if (1 == SCI2S1_RDRF){
    dummy = SCI2S1;
    buff[buff_index] = tmp;
    buff_index ++;
    if(200 == buff_index) {
      buff_index = 0;
      printf("String too length, some data may be lost !");
    }
    
  }
  
  
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : sci2_PutChar
* Returned Value   :
* Comments         :
*                     This function sends a char via SCI.
*
*END*----------------------------------------------------------------------*/
void sci2_PutChar(char send) 
{
  char dummy;
  
  while(!SCI2S1_TDRE);
  dummy = SCI2S1;
  SCI2D  = send;    
}


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : Initialize UART
* Returned Value   :
* Comments         :
*                    This function initializes the UART for console printf/scanf
*
*END*----------------------------------------------------------------------*/
UARTError InitializeUART(UARTBaudRate baudRate)
{
 uint_32 	baud_divisor;
 
 /* Calculate baud settings */
 /* baud_divisor = clock_speed/ baudrate + 0.5 */
 baud_divisor = (BUS_CLOCK + (8 * (uint_32)baudRate)) / (16 * (uint_32)baudRate);
  
 if (baud_divisor > 0x1fff) 
 {
  return kUARTUnknownBaudRate;
 }
  
      
 SCI1BDH = (uchar)((baud_divisor >> 8) & 0xFF);
 SCI1BDL = (uchar)(baud_divisor & 0xFF);

 return kUARTNoError;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : WriteUARTN
* Returned Value   :
* Comments         :
*                    This function writes N bytes on the SCI1 console output
*
*END*----------------------------------------------------------------------*/
UARTError WriteUARTN(const void* bytes, unsigned long length) 
{
  int i;
  char* src = (char*)bytes;
  
  for(i = 0; i< length; i++) 
  {
   TERMIO_PutChar(*src++);
  }

 return kUARTNoError;  
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : ReadUARTN
* Returned Value   :
* Comments         :
*                    This function reads N bytes on the SCI1 console input
*
*END*----------------------------------------------------------------------*/
UARTError ReadUARTN(void* bytes, unsigned long length)
{
  int i;
  char *dst = (char*)bytes;
  
  for(i = 0; i< length; i++) 
  {
   *dst++ = TERMIO_GetChar();
  }
  
  return kUARTNoError;
}