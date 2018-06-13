/*
 * File:		CAN_LOOPBACK_NODE.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "kinetis_flexcan.h"
#include "can.h"

// Prototypes
void  InitChip(void);
void  InitTarget(void);
void  CAN_Loopback(void);


/********************************************************************/
void main (void)
{
	char ch;
        
        // Init Chip
        InitChip();
    
        // Initialize target board
        InitTarget();        
        
        // Print Startup banners and options
        printf("\r\nHello Kinetis,Welcome to CAN Loopback Node!\r\n");

        // Loop back all messages to the remote node
        CAN_Loopback();
}





void  CAN_Loopback(void)
{
        char ch;
        HANDLE h;
        uint32  baud = 83;  // 83KHz
        uint32  rxMsgID = 0x82;
        uint32  txMsgID = 0x82;
        uint8   prio = 0;
        int16   noBytesRx = 0;
        uint32  idRxed;
        uint8   bytes[9];
                            
        // Initialize driver
        h = FlexCAN_Open();
        
        // Initialize CAN module
        FlexCAN_Init(baud);
       
        FlexCAN_EnableMB_Interrupt(FLEXCAN_RX_MB_START);    // enable rx MB interrupt
        FlexCAN_EnableMB_Interrupt(FLEXCAN_TX_MB_START);    // enable tx MB interrupt
        
        FlexCAN_ConfigureRxMBMask(FLEXCAN_RX_MB_START,0);   // receive all messages
        
        // Configure FLEXCAN_RX_MB_START MB as rx MB with rxMsgID
        FlexCAN_PrepareRxCANMB(FLEXCAN_RX_MB_START, rxMsgID);   
        
	while(1)
	{                
                // Read message bytes
                do{                
                 noBytesRx = FlexCAN_Read(h,0,&idRxed,bytes);
                }
                while(!noBytesRx);
                
                // echo the message to the remote node
                
                // Write to FlexCAN
                txMsgID = idRxed;
                FlexCAN_Write(h, txMsgID, 1,bytes,noBytesRx);           	         
	} 
}

void  InitTarget(void)
{   
#if (defined(TWR_K40X256))
  #define CAN1_USE_PTE24_25
#elif (defined(TWR_K60N512))
  #define CAN1_USE_PTE24_25
#else
  #define CAN1_USE_PTC16_17
#endif  
        // Configure pins
#ifdef  USE_FLEXCAN0  
  #define CAN0_USE_PTB18_19
          
  #ifdef  CAN0_USE_PTA12_13        
          // Configure CAN_RX/TX pins mulxed with PTA13/12 for FlexCAN0
          PORTC_PCR12 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP
          PORTC_PCR13 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP  
  #endif
  #ifdef  CAN0_USE_PTB18_19
          // Configure CAN_RX/TX pins mulxed with PTB18/19 for FlexCAN0
          PORTB_PCR18 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP
          PORTB_PCR19 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP  
  #endif   
#else
  //#define CAN1_USE_PTC16_17
  //#define CAN1_USE_PTE24_25
#ifdef  CAN1_USE_PTE24_25
          // Configure CAN_RX/TX pins mulxed with PTE24/25 for FlexCAN1
         PORTE_PCR24 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP
         PORTE_PCR25 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP   
            
  #endif 
  #ifdef  CAN1_USE_PTC16_17
          // Configure CAN_RX/TX pins mulxed with PTC16/17 for FlexCAN1
          PORTC_PCR16 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP
          PORTC_PCR17 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // PULLUP            
  #endif
          // Disable clock gating to PortC since other pins are not used
          SIM_SCGC5 &= ~(0x0800);
#endif                      
}



void InitChip(void)
{
  // It is already done in sysinit.c and start.c
  // Enable ERCLK clock to FlexCAN
  OSC_CR |= OSC_CR_ERCLKEN_MASK | OSC_CR_EREFSTEN_MASK;  
}


/********************************************************************/