
#include "common.h"
#include "crc.h"

// Macros
#define MAX_CHARS   256

// Globals
uint8_t  strMsg[MAX_CHARS];
uint8_t  isCRC16 = 1;
uint32_t polyTab[4] = 
{
0x1021,
0x8408,
0x8005,
0x04C11DB7L  
};

// Prototypes
static uint32_t  Input_Message(uint8_t *MsgBuf);
static uint32_t  ConvertASCII2Word(uint8_t *s4ASCII);
static uint32_t  Input_Word(void);

void main(void)
{
  int crc_error=0;
  uint32_t poly;
  uint32_t tot;
  uint32_t totr;
  uint32_t fxor;
  uint32_t tcrc;  
  uint8_t  gbQuit = 0;
  uint32_t  bByteCount;
  uint8_t  ch;
  uint32   seed,crc;

   // Enable clock gating to CRC
   SIM_SCGC6 |= SIM_SCGC6_CRC_MASK;
   
   isCRC16 = 1;
   printf("\r\n====== CRC Lab ==========\r\n");
 
   do
   { 
     printf("\r\nPlease select CRC width (16-bit/32-bit):\r\n");
     printf("1. CRC16\r\n");
     printf("2. CRC32\r\n");
     printf("select:");
     do{
       ch = in_char();
     }while ((ch != '1') && (ch != '2'));
     printf("%c\r\n",ch);
     isCRC16 = !(ch-'1');
 
     printf("\r\nPlease select CRC polynomial:\r\n");
     printf("1. poly = 0x1021 (CRC-CCITT)\r\n");
     printf("2. poly = 0x8408 (XMODEM)\r\n");
     printf("3. poly = 0x8005 (ARC)\r\n");
     printf("4. poly = 0x04C11DB7 (CRC32) \r\n");
     printf("5. others\r\n");
     printf("select:");
     do{
       ch = in_char();
     }while(!((ch < '6') && (ch > '0')));
     printf("%c\r\n",ch);
     if(ch == '5')
     {
       printf("Please enter a polynomial in hex format:"); 
       poly = Input_Word();
     }
     else
     {
       poly = polyTab[ch-'1'];
     }
     tcrc = (isCRC16)? 0: 1;                // width of CRC   
     
     printf("\r\nPlease select type of Transpose for input:\r\n");
     printf("1. No Transposition\r\n");
     printf("2. Only transpose bits in a byte\r\n");
     printf("3. Transpose both bits and bytes\r\n");
     printf("4. Only transpose bytes\r\n");
     printf("select:");
     do{
       ch = in_char();
     }while (!((ch < '5') && (ch > '0')));
     printf("%c\r\n",ch);
     tot = ch - '1';

     printf("\r\nPlease select type of Transpose for Read:\r\n");
     printf("1. No Transposition\r\n");
     printf("2. Only transpose bits in a byte\r\n");
     printf("3. Transpose both bits and bytes\r\n");
     printf("4. Only transpose bytes\r\n");
     printf("select:");
     do{
       ch = in_char();
     }while (!((ch < '5') && (ch > '0')));
     printf("%c\r\n",ch);
     totr = ch - '1';
     
     printf("XOR final checksum (y/n)?");
     do{
       ch = in_char();
     }while ((ch != 'y') && (ch != 'n'));
     out_char(ch);  
     printf("\r\n");
     fxor = (ch == 'y')? 1: 0;
   
      printf("Please enter seed in hex:");
      seed = Input_Word();
      printf("\r\nPlease enter an ASCII Message:");
      bByteCount = Input_Message(&strMsg[0]);
      crc_error=CRC_Config(poly,tot,totr,fxor,tcrc);  
       if(isCRC16)
      {
        crc = CRC_Cal_16(seed, &strMsg[0], bByteCount);
        printf("\r\n CRC result = %#04.4x\r\n",(crc & 0x0000FFFFL));
      }
      else
      {
        crc = CRC_Cal_32(seed, &strMsg[0], bByteCount);
        printf("\r\n CRC result = %#08.8x\r\n",crc);        
      }
      printf("Press any key to continue...,'q' to quit!\r\n");
      do
      {
        ch = in_char();
      }while( !ch );
      if(ch == 'q')
      {
        gbQuit = 1;
      }           
   }while(!gbQuit);
    
  gbQuit = 0;
  printf("Demo is quited!\r\n");
}

/*********************************************************************
 * Input_Message:  
 *   get a message from terminal. The message is terminated by ENTER key.
 *   The maximum number of characters in the message is defined by 
 *   MAX_CHARS which is 256.
\*********************************************************************/

uint32_t  Input_Message(uint8_t *MsgBuf)
{
  uint8_t ch;
  uint32_t count;
  
  count = 0;
  do
  {
    ch = in_char();
    out_char(ch);
    if(ch == '\r')
    {     
      break;
    }
    MsgBuf[count++] = ch; 
    if(count >= MAX_CHARS)
    {
      break;
    }
  }while(1);
    
  return (count);
}

/*********************************************************************
 * Input_Word:  
 *   get a word in hex from Terminal. word width is 4 digits. 
\*********************************************************************/

uint32_t  Input_Word(void)
{
  uint8_t ch;
  uint8_t count;
  uint8_t buf[8];
  uint32_t  w;
  int   i;
  
  count = 0; w = 0;
  for(i= 0;i< 8;i++)
  {
    buf[i] = 0;
  }
  do
  {
    ch = in_char();
    if(ch == '\r')
    {     
      out_char(ch);               // echo the char        
      break;
    }
    if( ((ch >= '0') && (ch <='9')) ||
        ((ch >= 'a') && (ch <='f')) ||
        ((ch >= 'A') && (ch <='F')))
    {
        
        buf[count++]  = ch;
        out_char(ch);               // echo the char        
    }
    if(count >= (isCRC16?4:8))
    {
      break;
    }
  }while(1);
    
  // Convert the ascii hex to an integer
  w = ConvertASCII2Word(buf);
  
  return (w);
  
}


/*********************************************************************
 * ConvertASCII2Word:  
 *   Convert an ASCII string to an integer word. word width is 4 digits. 
\*********************************************************************/

uint32_t  ConvertASCII2Word(uint8_t *s4ASCII)
{
    int   i;
    uint8_t  ch;
    uint32_t  w;
    w = 0;
    for(i = 0;i <= (isCRC16?3:7);i++)  
    {
        ch = s4ASCII[i];
        if(!ch)
        {
          break;
        }
        if((ch >= '0') && (ch <='9'))
        {
            ch -= '0';        
        } else if((ch >= 'a') && (ch <='f'))
        {
            ch = ch - 'a' + 10;
        }else if((ch >= 'A') && (ch <='F'))
        {
            ch = ch - 'A' + 10;
        }
        w <<= 4;
        w += ch;
    }
    
    return (w);
}