/*
 * File:		crc.c
 * Purpose:		crc algorithm routine for CRC16 and CRC32
 * Description:
 *
 * 
 */


#include "common.h"
#include "crc.h"

//#define BYTE_ENABLES_7_E
//#define BYTE_ENABLES_3_6_C
//#define BYTE_ENABLES_1_2_4_8

/********************************************************************/


    
 /********************************************************************/
int CRC_Config (uint32_t poly,uint32_t tot,uint32_t totr,uint32_t fxor,uint32_t tcrc)
{
  uint32_t ctrl_reg;
  int error = CRC_ERR_SUCCESS;
  
  // Configure CRC_CTRL Register
  if( tot > 3 ) {
  	error = CRC_ERR_TOT_VAL;
  }
  else if( totr > 3 ) {
  	error = CRC_ERR_TOTR_VAL;
  }
  else if( fxor > 1 ) {
  	error = CRC_ERR_FXOR_VAL;
  }
  else if( tcrc > 1 ) {
  	error = CRC_ERR_TCRC_VAL;
  } 
  else {
  	ctrl_reg=(tot<<30)|(totr<<28)|(fxor<<26)|(tcrc<<24);
  	CRC_CTRL=ctrl_reg;
  }
  
  // Configure Polynomial Register
  CRC_GPOLY=poly;

 return(error);
}


 /********************************************************************  
  * 16bit data CRC Calculation
 ********************************************************************/  
uint32_t CRC_Cal_16(uint32_t seed,uint8_t *msg, uint32_t sizeBytes)
{
  uint32_t ctrl_reg,data_out,data_in;
  uint8_t  *pCRCBytes;
  uint32_t sizeWords;
  uint32_t i,j;
  

  // Input seed, Set WaS=1
  ctrl_reg=CRC_CTRL;
  CRC_CTRL=ctrl_reg | 0x02000000;
  CRC_CRC=seed;
  
  // Input data, Set WaS=0
  CRC_CTRL=ctrl_reg & 0xFD000000;

  // Wait for calculation completion
  sizeWords = sizeBytes>>1;
  j = 0;
  for(i=0;i<sizeWords;i++){ 
      data_in = (msg[j] << 8) | (msg[j+1]);
      j += 2;
      CRC_CRC_LOW=data_in; 
  }
  if(j<sizeBytes)
  { 
     pCRCBytes = (uint8_t*)&CRC_CRC;
     *pCRCBytes++ = msg[j];
  }
  data_out=CRC_CRC;

 return(data_out);
}
 /********************************************************************
  * 32bit data CRC Calculation
 ********************************************************************/  
uint32_t CRC_Cal_32(uint32_t seed,uint8_t *msg, uint32_t sizeBytes)
{
  uint32_t ctrl_reg,data_out,data_in;
  uint32_t sizeDwords;
  uint8_t  *pCRCBytes;
  int i,j;
  // Input seed, Set WaS=1
  ctrl_reg=CRC_CTRL;
  CRC_CTRL=ctrl_reg|0x02000000;
  CRC_CRC=seed;
 
  // Input data, Set WaS=0
  CRC_CTRL=ctrl_reg&0xFD000000;
  
  // Wait for calculation completion
  sizeDwords = sizeBytes>>2;
  j = 0;
  for(i=0;i<sizeDwords;i++){ 
      data_in = ((msg[j] << 24) | (msg[j+1] << 16) | (msg[j+2] << 8) | msg[j+3]);
      j += 4;
      CRC_CRC = data_in; 
  }
  if(j<sizeBytes)
  {
    pCRCBytes = (uint8_t*)&CRC_CRC;

#if  defined(BYTE_ENABLES_1_2_4_8)   // write single byte  
    for(;j<sizeBytes;j++)
    {     
       *pCRCBytes++ = msg[j];
    }
#elif  defined(BYTE_ENABLES_3_6_C)      // write two bytes
    data_in = 0;
    i = 0;
    for(;j<sizeBytes;j++)
    {     
      data_in = (data_in <<8) | msg[j];
      i++;
      if(i==2)
      {
        i = 0;
        CRC_CRC_LOW = data_in;
      }
    }
    if(i==1)
    {
       CRC_CRC_LL = data_in; // write last byte
    }
#elif  defined(BYTE_ENABLES_7_E)        // write three bytes
    data_in = 0;
    i = 0;
    for(;j<sizeBytes;j++)
    {     
      data_in = (data_in <<8) | msg[j];
      i++;
      if(i==3)
      {
        i = 0;
        // write first  char
        CRC_CRC_HL  = (data_in>>16) & 0xff;   // write low byte of high word
        // write last two chars
        CRC_CRC_LOW = data_in & 0x00ffff;     // write low word
       }
    }
    if( i == 2)
    {
       CRC_CRC_LOW = (data_in); // write last byte
    }
    else if (i == 1)
    {
       CRC_CRC_LL = data_in; // write last byte      
    }
#else                          // write low byte only
    for(;j<sizeBytes;j++)
    {     
       *pCRCBytes = msg[j];
    }
#endif            
  }
  data_out=CRC_CRC;

 return(data_out);
}
 /********************************************************************/ 
