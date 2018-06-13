/*
 * File:		flexbus.c
 * Purpose:		Example of communicating to an MRAM with the FlexBus
 *              See the readme file for instructions
 */

#include "common.h"

void TWRK40_flexbus_init(void);
void TWRK60_flexbus_init(void);

#define MRAM_START_ADDRESS	(*(vuint8*)(0x60000000))

/********************************************************************/
void main (void)
{
    uint8 wdata8 = 0x00;
    uint8 rdata8 = 0x00;
    uint16 wdata16 = 0x00;
    uint16 rdata16 = 0x00;
    uint32 wdata32 = 0x00;
    uint32 rdata32 = 0x00;
    uint32 n = 0;

    printf("\n****MRAM Test****\n");

    printf("\nInitializing the FlexBus\n");
 
    #if ( defined(TWR_K40X256) | defined(TWR_K53N512) )
    /* The TWR_K40X256 and TWR_K53N512 use the same FlexBus pins so the
     * K40 tower init can be reused for the K53 tower.
     */
      TWRK40_flexbus_init(); 
    #elif TWR_K60N512 
      TWRK60_flexbus_init();
    #else
      printf("\nError - Board not defined \n");  
    #endif
    
    //print FlexBus configuration register contents
    printf("FB_CSCR0 is %08x \n\r",FB_CSCR0);
    printf("FB_CSMR0 is %08x \n\r",FB_CSMR0);
    printf("FB_CSAR0 is %08x \n\r",FB_CSAR0);
    printf("FB_CSPMCR is %08x \n\r",FB_CSPMCR);
    printf("SIM_CLKDIV1 is %08x \n\r",SIM_CLKDIV1);

	printf("\nTesting 8-bit write/reads\n");

   	wdata8=0xA5;   //data to write to mram

	for(n=0x00000;n<0x000F;n++)  //address offset
	{
  		*(vuint8*)(&MRAM_START_ADDRESS + n) = wdata8;  //write
  		rdata8=0x00;  //clear data variable;
  		rdata8=(*(vuint8*)(&MRAM_START_ADDRESS + n));  //read 
  		printf("ADDR: 0x%08x WRITE: 0x%02x READ: 0x%02x\n",&MRAM_START_ADDRESS + n,wdata8,rdata8);
	}

	printf("\nTesting 16-bit write/reads\n");

   	wdata16=0x1234;   //data to write to mram

	for(n=0x00010;n<0x001F;n+=2)  //address offset
	{
  		*(vuint16*)(&MRAM_START_ADDRESS + n) = wdata16;  //write
  		rdata16=0x00;  //clear data variable;
  		rdata16=(*(vuint16*)(&MRAM_START_ADDRESS + n));  //read
		printf("ADDR: 0x%08x WRITE: 0x%04x READ: 0x%04x\n",&MRAM_START_ADDRESS + n,wdata16,rdata16);
	}

	printf("\nTesting 32-bit write/reads\n");

   	wdata32=0x87654321;   //data to write to mram

	for(n=0x00020;n<0x002F;n+=4)  //address offset
	{
  		*(vuint32*)(&MRAM_START_ADDRESS + n) = wdata32;  //write
  		rdata32=0x00;  //clear data variable;
  		rdata32=(*(vuint32*)(&MRAM_START_ADDRESS + n));  //read
 		printf("ADDR: 0x%08x WRITE: 0x%08x READ: 0x%08x\n",&MRAM_START_ADDRESS + n,wdata32,rdata32);
	}

}
/********************************************************************/


void TWRK40_flexbus_init(void){

/* Enable the FlexBus                                               */
/* Configure the FlexBus Registers for 8-bit port size              */
/*  with multiplexed address and data using chip select 0           */
/* These configurations are specific to communicating with          */ 
/*  the MRAM used in this example                                   */
/* For K40 tower module - use the byte lane shift because there     */  
/*  is a latch on the board which handles multiplexed address/data  */
  
  //Set Base address
  FB_CSAR0 = (uint32)&MRAM_START_ADDRESS;

  FB_CSCR0  =   FB_CSCR_BLS_MASK   // set byte lane shift for data on FB_AD[7:0] aka. right justified mode
              | FB_CSCR_PS(1)      // 8-bit port
              | FB_CSCR_AA_MASK    // auto-acknowledge
              | FB_CSCR_ASET(0x1)  // assert chip select on second clock edge after address is asserted
              | FB_CSCR_WS(0x1)    // 1 wait state - may need a wait state depending on the bus speed 
              ;

  FB_CSMR0  =   FB_CSMR_BAM(0x7)  //Set base address mask for 512K address space
              | FB_CSMR_V_MASK    //Enable cs signal
              ;
              
  //enable BE signals - note, not used in this example
  FB_CSPMCR = 0x02200000;

  //fb clock divider 3
   SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV3(0x3);
  
  /* Configure the pins needed to FlexBus Function (Alt 5) */
  /* this example uses low drive strength settings         */
  //address/Data
  PORTA_PCR7=PORT_PCR_MUX(5);          //fb_ad[18]
  PORTA_PCR8=PORT_PCR_MUX(5);          //fb_ad[17]
  PORTA_PCR9=PORT_PCR_MUX(5);          //fb_ad[16]
  PORTA_PCR10=PORT_PCR_MUX(5);          //fb_ad[15]
  PORTA_PCR24=PORT_PCR_MUX(5);          //fb_ad[14]
  PORTA_PCR25=PORT_PCR_MUX(5);          //fb_ad[13]
  PORTA_PCR26=PORT_PCR_MUX(5);          //fb_ad[12]
  PORTA_PCR27=PORT_PCR_MUX(5);          //fb_ad[11]
  PORTA_PCR28=PORT_PCR_MUX(5);          //fb_ad[10]
  PORTD_PCR10=PORT_PCR_MUX(5);          //fb_ad[9]
  PORTD_PCR11=PORT_PCR_MUX(5);          //fb_ad[8]
  PORTD_PCR12=PORT_PCR_MUX(5);          //fb_ad[7]
  PORTD_PCR13=PORT_PCR_MUX(5);          //fb_ad[6]
  PORTD_PCR14=PORT_PCR_MUX(5);          //fb_ad[5]
  PORTE_PCR8=PORT_PCR_MUX(5);          //fb_ad[4]
  PORTE_PCR9=PORT_PCR_MUX(5);          //fb_ad[3]
  PORTE_PCR10=PORT_PCR_MUX(5);          //fb_ad[2]
  PORTE_PCR11=PORT_PCR_MUX(5);          //fb_ad[1]
  PORTE_PCR12=PORT_PCR_MUX(5);          //fb_ad[0]
  //control signals
  PORTA_PCR11=PORT_PCR_MUX(5);          //fb_oe_b
  PORTD_PCR15=PORT_PCR_MUX(5);          //fb_rw_b
  PORTE_PCR7=PORT_PCR_MUX(5);          //fb_cs0_b
  PORTE_PCR6=PORT_PCR_MUX(5);          //fb_ale  
}

void TWRK60_flexbus_init(void){

/* Enable the FlexBus                                               */
/* Configure the FlexBus Registers for 8-bit port size              */
/*  with separate address and data using chip select 0              */
/* These configurations are specific to communicating with          */ 
/*  the MRAM used in this example                                   */
/* For K60 tower module - do not set byte lane shift so that data   */
/*  comes out on AD[31:24]                                          */
  
  //Set Base address
  FB_CSAR0 = (uint32)&MRAM_START_ADDRESS;

  FB_CSCR0  =   FB_CSCR_PS(1)      // 8-bit port
              | FB_CSCR_AA_MASK    // auto-acknowledge
              | FB_CSCR_ASET(0x1)  // assert chip select on second clock edge after address is asserted
              | FB_CSCR_WS(0x1)    // 1 wait state - may need a wait state depending on the bus speed 
              ;

  FB_CSMR0  =   FB_CSMR_BAM(0x7)  //Set base address mask for 512K address space
              | FB_CSMR_V_MASK    //Enable cs signal
              ;
              
   //enable BE signals - note, not used in this example
  FB_CSPMCR = 0x02200000;

  //fb clock divider 3
  SIM_CLKDIV1 |= SIM_CLKDIV1_OUTDIV3(0x3);
   
   
  /* Configure the pins needed to FlexBus Function (Alt 5) */
  /* this example uses low drive strength settings         */

  //address
  PORTB_PCR11 = PORT_PCR_MUX(5);           //  fb_ad[18]
  PORTB_PCR16 = PORT_PCR_MUX(5);           //  fb_ad[17]
  PORTB_PCR17 = PORT_PCR_MUX(5);           //  fb_ad[16]
  PORTB_PCR18 = PORT_PCR_MUX(5);           //  fb_ad[15]
  PORTC_PCR0  = PORT_PCR_MUX(5);           //  fb_ad[14]
  PORTC_PCR1  = PORT_PCR_MUX(5);           //  fb_ad[13]
  PORTC_PCR2  = PORT_PCR_MUX(5);           //  fb_ad[12]
  PORTC_PCR4  = PORT_PCR_MUX(5);           //  fb_ad[11]
  PORTC_PCR5  = PORT_PCR_MUX(5);           //  fb_ad[10]
  PORTC_PCR6  = PORT_PCR_MUX(5);           //  fb_ad[9]
  PORTC_PCR7  = PORT_PCR_MUX(5);           //  fb_ad[8]
  PORTC_PCR8  = PORT_PCR_MUX(5);           //  fb_ad[7]
  PORTC_PCR9  = PORT_PCR_MUX(5);           //  fb_ad[6]
  PORTC_PCR10 = PORT_PCR_MUX(5);           //  fb_ad[5]
  PORTD_PCR2  = PORT_PCR_MUX(5);           //  fb_ad[4]
  PORTD_PCR3  = PORT_PCR_MUX(5);           //  fb_ad[3]
  PORTD_PCR4  = PORT_PCR_MUX(5);           //  fb_ad[2]
  PORTD_PCR5  = PORT_PCR_MUX(5);           //  fb_ad[1]
  PORTD_PCR6  = PORT_PCR_MUX(5);           //  fb_ad[0]
  
  //data
  PORTB_PCR20 = PORT_PCR_MUX(5);           //  fb_ad[31] used as d[7]
  PORTB_PCR21 = PORT_PCR_MUX(5);           //  fb_ad[30] used as d[6]
  PORTB_PCR22 = PORT_PCR_MUX(5);           //  fb_ad[29] used as d[5]
  PORTB_PCR23 = PORT_PCR_MUX(5);           //  fb_ad[28] used as d[4]
  PORTC_PCR12 = PORT_PCR_MUX(5);           //  fb_ad[27] used as d[3]
  PORTC_PCR13 = PORT_PCR_MUX(5);           //  fb_ad[26] used as d[2]
  PORTC_PCR14 = PORT_PCR_MUX(5);           //  fb_ad[25] used as d[1]
  PORTC_PCR15 = PORT_PCR_MUX(5);           //  fb_ad[24] used as d[0]

  //control signals
  PORTB_PCR19 = PORT_PCR_MUX(5);          // fb_oe_b
  PORTC_PCR11 = PORT_PCR_MUX(5);          // fb_rw_b  
  PORTD_PCR1  = PORT_PCR_MUX(5);          // fb_cs0_b
  PORTD_PCR0  = PORT_PCR_MUX(5);          // fb_ale

}