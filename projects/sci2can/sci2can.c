/*
 * File:		SCI2CAN.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "kinetis_flexcan.h"
#include "can.h"


// Prototypes
void  InitChip(void);
void  InitTarget(void);
void  RxFIFO_Demo(void);
void  SCI2CAN_Demo(void);

/********************************************************************/
void main (void)
{
	char ch;
        
        // Init Chip
        InitChip();
    
        // Initialize target board
        InitTarget();        
        
        do
        {
          // Print Startup banners and options
          printf("\r\nHello Kinetis,Welcome to FlexCAN demo labs!\r\n");
          printf("Please select one demo below:\r\n");
          printf("1. SCI2CAN bridge\r\n");
          printf("2. Rx FIFO demo\r\n");
          printf("Select:");
          do{
              ch = in_char();
          }while( (ch != '1') && (ch != '2') );
          out_char(ch);
          if(ch=='1')
          {
              SCI2CAN_Demo();
          }
          else
          {
              RxFIFO_Demo();
          }
        }while (1);
}


void  RxFIFO_Demo(void)
{
        char ch;
        HANDLE h;
 //       uint32  baud = 1000;  // 1MHz
        uint32  baud = 83;  // 83KHz
        uint32  rxMsgID = 0x82;
        uint32  txMsgID = 0x82;
        uint8   prio = 0;
        int16   noBytesRx = 0;
        uint32  idRxed;
        uint16  iTxMBNo;
        uint8   bytes[8];
        uint8   bytesRx[9];          
        uint32  idList[FLEXCAN_NO_RXFIFO_FILTERS+1];
        int     i;
        int     msgCountRxed;
        
  	printf("\nWelcome to RxFIFO demo!\n");
        printf("It will transmit 9 messages, and receive them into Rx FIFO and 1 MB.\r\n");
               
        // Initialize data bytes
        for(i=0; i < 8; i++)
        {
          bytes[i] = 0x50+i;
          bytesRx[i] = 0;
        }
        bytesRx[i] = 0;
        
        // Initialize driver
        h = FlexCAN_Open();
        
        // Initialize CAN module
        FlexCAN_Init(baud);
        
        // Configure RxFIFO filter table elements
        for(i = 0; i < FLEXCAN_NO_RXFIFO_FILTERS; i++)
        {
           idList[i] = (i+1) | CAN_MSG_IDE_MASK;  // extended ID
        }
        rxMsgID = (i+1);  //  standard ID
        FlexCAN_ConfigureRxFIFO_IDTable(idList,8);    // RFFN = 0, 8 ID filter table elements
                                                      // as defined by FLEXCAN_NO_RXFIFO_FILTERS
        
        // Configure RxFiFO filter mask
        for(i=0; i<FLEXCAN_NO_RXFIFO_FILTERS;i++)
        {
          FlexCAN_ConfigureRxFIFOFilterTabElementMask(i,0xffffffffL); // match all ID bits
        }
        
        // Now enable all interrupts for rx MB, tx MB and rx FIFO
        for(iTxMBNo = 8; iTxMBNo < NUMBER_OF_MB; iTxMBNo++)
        {           
          FlexCAN_EnableMB_Interrupt(iTxMBNo);
        }
        FlexCAN_EnableMB_Interrupt(5);    // enable rx FIFO interrupt
        
        // Configure  MB8 as rx MB with rxMsgID as MB0 to 7 are used as Rx FIFO
        FlexCAN_PrepareRxCANMB(8, rxMsgID);   

        
        // Now send 9 messages via MB9, MB10 to MB15
        iTxMBNo = 9;
        for(i = 0; i < 7; i++)
        {
            txMsgID = (i+1) | CAN_MSG_IDE_MASK;  // extended ID
            FlexCAN_PrepareAndStartTransmit(iTxMBNo++,txMsgID,i,bytes, 8,FLEXCAN_MB_CODE_TX_ONCE);
            bytes[7]++;
        }
        iTxMBNo = 9;  // wrap around 
        for(i = 7; i < 9; i++)
        {              
              while(!FlexCAN_Is_Transfer_Done(iTxMBNo));  // wait transfer to be done                
              if(i != 8)
              {
                txMsgID = (i+1) | CAN_MSG_IDE_MASK;  // extended ID
              }
              else
              {              
                txMsgID = (i+1);  // standard ID
              }    
            FlexCAN_PrepareAndStartTransmit(iTxMBNo++,txMsgID,i,bytes, 8,FLEXCAN_MB_CODE_TX_ONCE); 
            bytes[7]++;            
        }
        
        msgCountRxed = 0;
        printf("============================================================================\r\n");
        // Wait for messages to be received.
	while(1)
	{
                // Read message bytes from Rx FIFO
                noBytesRx = FlexCAN_Read_RxFIFOQueue(h,&idRxed,bytesRx);
                if(noBytesRx >0)
                {
                  printf("Message Received in RxFIFO: ID=%#08.8x",idRxed & 0x7FFFFFFFL);
                  if(idRxed & CAN_MSG_IDE_MASK)
                  {
                    printf(" (Extended format)");
                  }
                  else
                  {
                    printf(" (Standard format)");
                  }
                  printf(", data[] = \r\n");
                  for(i=0; i< noBytesRx; i++)
                  {
                     printf(" %#04.4x ",bytesRx[i]);                         
                  }
                  printf("\r\n\r\n");
                  msgCountRxed++;
                }
                // Read message bytes from Rx MB              
                noBytesRx = FlexCAN_Read(h,8,&idRxed,bytesRx);
                if(noBytesRx>0)
                {
                  printf("Message Received in MB[%d]: ID=%#08.8x",8,idRxed & 0x7FFFFFFFL);
                  if(idRxed & CAN_MSG_IDE_MASK)
                  {
                    printf(" (Extended format)");
                  }
                  else
                  {
                    printf(" (Standard format)");
                  }
                  printf(", data[] = \r\n");
                  for(i=0; i< noBytesRx; i++)
                    {
                          printf(" %#04.4x ",bytesRx[i]);                         
                    }
                    printf("\r\n\r\n");                  
                   msgCountRxed++;
                }
                if(msgCountRxed==9)
                {
                  break;
                }
         }     
        printf("============================================================================\r\n");
        printf("Congratulations: you've completed Rx FIFO demo!\r\n\r\n");
}


void  SCI2CAN_Demo(void)
{
        char ch;
        HANDLE h;
//        uint32  baud = 1000;  // 1MHz
        uint32  baud = 83;  // 83KHz
        uint32  rxMsgID = 0x82;
        uint32  txMsgID = 0x82;
        uint8   prio = 0;
        int16   noBytesRx = 0;
        uint32  idRxed;
        uint8   bytes[9];
        
  	printf("\nWelcome to SCI_to_CAN Bridge demo!\n");
                     
        // Initialize driver
        h = FlexCAN_Open();
        
        // Initialize CAN module
        FlexCAN_Init(baud);
       
        FlexCAN_EnableMB_Interrupt(FLEXCAN_RX_MB_START);    // enable rx MB interrupt
        FlexCAN_EnableMB_Interrupt(FLEXCAN_TX_MB_START);    // enable tx MB interrupt
        
        // Configure FLEXCAN_RX_MB_START MB as rx MB with rxMsgID
        FlexCAN_PrepareRxCANMB(FLEXCAN_RX_MB_START, rxMsgID);   

        printf("\nPlease enter any character to be sent to remote node...\n");
        
	while(1)
	{
		ch = in_char();
                
                if(ch == '\x11')  // Control-Q, DC1, also known as XON 
                {
                  break;
                }
                // Write to FlexCAN
                FlexCAN_Write(h, txMsgID, prio, (uint8 *)&ch,1);           
                
                // Read message bytes
                do{                
                 noBytesRx = FlexCAN_Read(h,0,&idRxed,bytes);
                }
                while(!noBytesRx);
                
                // echo the char to local terminal
		out_char(bytes[0]);
                //bytes[noBytesRx] = 0;
                //printf("%s\n",bytes);                         
	} 
        printf("============================================================================\r\n");
        printf("Congratulations: you've completed SCI2CAN bridge demo!\r\n\r\n");
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
