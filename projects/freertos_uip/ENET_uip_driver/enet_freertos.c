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
	See http://www.FreeRTOS.org for documentation, latest information, license
	and contact details.  Please ensure to read the configuration and relevant
	port sections of the online documentation.
	***************************************************************************
*/

/* Kernel includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "common.h"

/* Demo includes. */
#include "eth_phy.h"
#include "enet.h"
#include "mii.h"

/*FSL: specific includes*/
#include "enet_freertos.h"

/* uIP includes. */
#include "uip.h"
#include "uip_arp.h"


/* PHY hardware specifics. */
#define PHY_STATUS								( 0x1F )
#define PHY_DUPLEX_STATUS							( 4<<2 )
#define PHY_SPEED_STATUS							( 1<<2 )

/* Delay between polling the PHY to see if a link has been established. */
#define enetLINK_DELAY								( 500 / portTICK_RATE_MS )

/* Very short delay to use when waiting for the Tx to finish with a buffer if
we run out of Rx buffers. */
#define enetMINIMAL_DELAY							( 2 / portTICK_RATE_MS )

/* The Tx re-uses the Rx buffers and only has one descriptor. */
#define enetNUM_TX_DESCRIPTORS					( 1 )
/*-----------------------------------------------------------*/

/* The semaphore used to wake the uIP task when data arrives. */
xSemaphoreHandle xENETSemaphore = NULL;

extern int periph_clk_khz;

/* The buffer used by the uIP stack.  In this case the pointer is used to
point to one of the Rx buffers to avoid having to copy the Rx buffer into
the uIP buffer. */
unsigned portCHAR *uip_buf;

/* The DMA descriptors.  These are char arrays to allow us to align them 
correctly. */
static unsigned portCHAR xENETTxDescriptors_unaligned[ ( enetNUM_TX_DESCRIPTORS * sizeof( NBUF ) ) + 16 ];
static unsigned portCHAR xENETRxDescriptors_unaligned[ ( configNUM_ENET_RX_BUFFERS * sizeof( NBUF ) ) + 16 ];
static NBUF *pxENETTxDescriptor;
static NBUF *xENETRxDescriptors;

/* The DMA buffer.  THis is a char arrays to allow it to be aligned correctly. */
static unsigned portCHAR ucENETRxBuffers[ ( configNUM_ENET_RX_BUFFERS * configENET_BUFFER_SIZE ) + 16 ];
static unsigned portBASE_TYPE uxNextRxBuffer = 0;

/*-----------------------------------------------------------*/

static void prvInitialiseENETBuffers( void )
{
unsigned portBASE_TYPE ux;
unsigned portCHAR *pcBufPointer;

	/* Set the pointer to a correctly aligned address. */
	pcBufPointer = &( xENETTxDescriptors_unaligned[ 0 ] );
	while( ( ( unsigned portLONG ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	
	pxENETTxDescriptor = ( NBUF * ) pcBufPointer;	

	/* Likewise the pointer to the Tx descriptor. */	
	pcBufPointer = &( xENETRxDescriptors_unaligned[ 0 ] );
	while( ( ( unsigned portLONG ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	
	xENETRxDescriptors = ( NBUF * ) pcBufPointer;

	/* There is no Tx buffer as the Rx buffer is reused. */
	pxENETTxDescriptor->length = 0;
	pxENETTxDescriptor->status = 0;
        
        #ifdef ENHANCED_BD
        pxENETTxDescriptor->ebd_status = TX_BD_IINS | TX_BD_PINS;
        #endif

	/* Align the Rx buffers. */
	pcBufPointer = &( ucENETRxBuffers[ 0 ] );
	while( ( ( unsigned portLONG ) pcBufPointer & 0x0fUL ) != 0 )
	{
		pcBufPointer++;
	}
	
	/* Then fill in the Rx descriptors. */
	for( ux = 0; ux < configNUM_ENET_RX_BUFFERS; ux++ )
	{
	    xENETRxDescriptors[ ux ].status = RX_BD_E;
	    xENETRxDescriptors[ ux ].length = 0;
	    #ifdef NBUF_LITTLE_ENDIAN
            xENETRxDescriptors[ ux ].data = (uint8_t *)__REV((uint32_t)pcBufPointer);
            #else
            xENETRxDescriptors[ ux ].data = pcBufPointer;
            #endif
	    pcBufPointer += configENET_BUFFER_SIZE;

            #ifdef ENHANCED_BD
	    xENETRxDescriptors[ ux ].bdu = 0x00000000;
	    xENETRxDescriptors[ ux ].ebd_status = RX_BD_INT;
            #endif 
	}

	/* Set the wrap bit in the last descriptors to form a ring. */
	xENETRxDescriptors[ configNUM_ENET_RX_BUFFERS - 1 ].status |= RX_BD_W;

	/* We start with descriptor 0. */
	uxNextRxBuffer = 0;
}
/*-----------------------------------------------------------*/

void vInitENET( void )
{
/*unsigned portLONG*/ int usData;
struct uip_eth_addr xAddr;
const unsigned portCHAR ucMACAddress[6] = 
{
	configMAC_ADDR0, configMAC_ADDR1,configMAC_ADDR2,configMAC_ADDR3,configMAC_ADDR4,configMAC_ADDR5
};
 
	/* Enable the ENET clock. */
        SIM_SCGC2 |= SIM_SCGC2_ENET_MASK;

        /*FSL: allow concurrent access to MPU controller. Example: ENET uDMA to SRAM, otherwise bus error*/
        MPU_CESR = 0;         
        
	prvInitialiseENETBuffers();

	/* Create the semaphore used to wake the uIP task when data arrives. */
	vSemaphoreCreateBinary( xENETSemaphore );
	
	/* Set the MAC address within the stack. */
	for( usData = 0; usData < 6; usData++ )
	{
		xAddr.addr[ usData ] = ucMACAddress[ usData ];
	}
	uip_setethaddr( xAddr );	   
    
	/* Set the Reset bit and clear the Enable bit */
	ENET_ECR = ENET_ECR_RESET_MASK;

	/* Wait at least 8 clock cycles */
	for( usData = 0; usData < 10; usData++ )
	{
		asm( "NOP" );
	}
    
        /*FSL: start MII interface*/
        mii_init(0, periph_clk_khz/1000/*MHz*/);       
        
        //enet_interrupt_routine
        set_irq_priority (76, 6);
        enable_irq(76);//ENET xmit interrupt
        //enet_interrupt_routine
        set_irq_priority (77, 6);
        enable_irq(77);//ENET rx interrupt
        //enet_interrupt_routine
        set_irq_priority (78, 6);
        enable_irq(78);//ENET error and misc interrupts           
        
	/*
	 * Make sure the external interface signals are enabled
	 */
        PORTB_PCR0  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDIO/MII0_MDIO
	PORTB_PCR1  = PORT_PCR_MUX(4);//GPIO;//RMII0_MDC/MII0_MDC		

#if configUSE_MII_MODE
	PORTA_PCR14 = PORT_PCR_MUX(4);//RMII0_CRS_DV/MII0_RXDV
        //PORTA_PCR5  = PORT_PCR_MUX(4);//RMII0_RXER/MII0_RXER
	PORTA_PCR12 = PORT_PCR_MUX(4);//RMII0_RXD1/MII0_RXD1
	PORTA_PCR13 = PORT_PCR_MUX(4);//RMII0_RXD0/MII0_RXD0
        PORTA_PCR15 = PORT_PCR_MUX(4);//RMII0_TXEN/MII0_TXEN
	PORTA_PCR16 = PORT_PCR_MUX(4);//RMII0_TXD0/MII0_TXD0
	PORTA_PCR17 = PORT_PCR_MUX(4);//RMII0_TXD1/MII0_TXD1
        PORTA_PCR11 = PORT_PCR_MUX(4);//MII0_RXCLK
        PORTA_PCR25 = PORT_PCR_MUX(4);//MII0_TXCLK
	PORTA_PCR9  = PORT_PCR_MUX(4);//MII0_RXD3
	PORTA_PCR10 = PORT_PCR_MUX(4);//MII0_RXD2	
	PORTA_PCR28 = PORT_PCR_MUX(4);//MII0_TXER
	PORTA_PCR24 = PORT_PCR_MUX(4);//MII0_TXD2
	PORTA_PCR26 = PORT_PCR_MUX(4);//MII0_TXD3
	PORTA_PCR27 = PORT_PCR_MUX(4);//MII0_CRS
	PORTA_PCR29 = PORT_PCR_MUX(4);//MII0_COL
#else
	PORTA_PCR14 = PORT_PCR_MUX(4);//RMII0_CRS_DV/MII0_RXDV
        //PORTA_PCR5  = PORT_PCR_MUX(4);//RMII0_RXER/MII0_RXER
	PORTA_PCR12 = PORT_PCR_MUX(4);//RMII0_RXD1/MII0_RXD1
	PORTA_PCR13 = PORT_PCR_MUX(4);//RMII0_RXD0/MII0_RXD0
        PORTA_PCR15 = PORT_PCR_MUX(4);//RMII0_TXEN/MII0_TXEN
	PORTA_PCR16 = PORT_PCR_MUX(4);//RMII0_TXD0/MII0_TXD0
	PORTA_PCR17 = PORT_PCR_MUX(4);//RMII0_TXD1/MII0_TXD1
#endif   
    
	/* Can we talk to the PHY? */
	do
	{
		vTaskDelay( enetLINK_DELAY );
		usData = 0xffff;
		mii_read( 0, configPHY_ADDRESS, PHY_PHYIDR1, &usData );
        
	} while( usData == 0xffff );

	/* Start auto negotiate. */
	mii_write( 0, configPHY_ADDRESS, PHY_BMCR, ( PHY_BMCR_AN_RESTART | PHY_BMCR_AN_ENABLE ) );

	/* Wait for auto negotiate to complete. */
	do
	{
		vTaskDelay( enetLINK_DELAY );
		mii_read( 0, configPHY_ADDRESS, PHY_BMSR, &usData );

	} while( !( usData & PHY_BMSR_AN_COMPLETE ) );

	/* When we get here we have a link - find out what has been negotiated. */
	usData = 0;
	mii_read( 0, configPHY_ADDRESS, PHY_STATUS, &usData );	

	/* Clear the Individual and Group Address Hash registers */
	ENET_IALR = 0;
	ENET_IAUR = 0;
	ENET_GALR = 0;
	ENET_GAUR = 0;
	
	/* Set the Physical Address for the selected ENET */
	enet_set_address( 0, ucMACAddress );
        
#if configUSE_MII_MODE        
	/* Various mode/status setup. */
	ENET_RCR = ENET_RCR_MAX_FL(configENET_BUFFER_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK;
#else
        ENET_RCR = ENET_RCR_MAX_FL(configENET_BUFFER_SIZE) | ENET_RCR_MII_MODE_MASK | ENET_RCR_CRCFWD_MASK | ENET_RCR_RMII_MODE_MASK;
#endif

        /*FSL: clear rx/tx control registers*/
        ENET_TCR = 0;
        
	/* Setup half or full duplex. */
	if( usData & PHY_DUPLEX_STATUS )
	{
		/*Full duplex*/
                ENET_RCR &= (unsigned portLONG)~ENET_RCR_DRT_MASK;
		ENET_TCR |= ENET_TCR_FDEN_MASK;
	}
	else
	{
		/*half duplex*/
                ENET_RCR |= ENET_RCR_DRT_MASK;
		ENET_TCR &= (unsigned portLONG)~ENET_TCR_FDEN_MASK;
	}
        /* Setup speed */
        if( usData & PHY_SPEED_STATUS )
        {
                /*10Mbps*/
                ENET_RCR |= ENET_RCR_RMII_10T_MASK;
        }

	#if( configUSE_PROMISCUOUS_MODE == 1 )
	{
		ENET_RCR |= ENET_RCR_PROM_MASK;
	}
	#endif

        #ifdef ENHANCED_BD
                ENET_ECR = ENET_ECR_EN1588_MASK;
        #else
                ENET_ECR = 0;
        #endif
        
	/* Set Rx Buffer Size */
	ENET_MRBR = (unsigned portSHORT) configENET_BUFFER_SIZE;

	/* Point to the start of the circular Rx buffer descriptor queue */
	ENET_RDSR = ( unsigned portLONG ) &( xENETRxDescriptors[ 0 ] );

	/* Point to the start of the circular Tx buffer descriptor queue */
	ENET_TDSR = ( unsigned portLONG ) pxENETTxDescriptor;

	/* Clear all ENET interrupt events */
	ENET_EIR = ( unsigned portLONG ) -1;

	/* Enable interrupts. */
	ENET_EIMR = ENET_EIR_TXF_MASK | ENET_EIMR_RXF_MASK | ENET_EIMR_RXB_MASK | ENET_EIMR_UN_MASK | ENET_EIMR_RL_MASK | ENET_EIMR_LC_MASK | ENET_EIMR_BABT_MASK | ENET_EIMR_BABR_MASK | ENET_EIMR_EBERR_MASK;

	/* Enable the MAC itself. */
        ENET_ECR |= ENET_ECR_ETHEREN_MASK;

        /* Indicate that there have been empty receive buffers produced */
        ENET_RDAR = ENET_RDAR_RDAR_MASK;
#if 0
        enet_debug_dump();
#endif
}
/*-----------------------------------------------------------*/

unsigned portLONG ulENETRx( void )
{
unsigned portLONG ulLen = 0UL;

	/* Is a buffer ready? */
	if( ( xENETRxDescriptors[ uxNextRxBuffer ].status & RX_BD_E ) == 0 )
	{
		/* Obtain the size of the packet and put it into the "len" variable. */
		#ifdef NBUF_LITTLE_ENDIAN
                ulLen =  __REVSH(xENETRxDescriptors[ uxNextRxBuffer ].length);
		uip_buf =  (uint8_t *)__REV((uint32_t)xENETRxDescriptors[ uxNextRxBuffer ].data);
                //not required by uIP
                #ifdef ENHANCED_BD
                //__REV(xENETRxDescriptors[ uxNextRxBuffer ].timestamp);
		//__REVSH(xENETRxDescriptors[ uxNextRxBuffer ].length_proto_type);
		//__REVSH(xENETRxDescriptors[ uxNextRxBuffer ].payload_checksum);
                #endif
                #else
                ulLen =  xENETRxDescriptors[ uxNextRxBuffer ].length;
		uip_buf =  xENETRxDescriptors[ uxNextRxBuffer ].data;
                //not required by uIP
                #ifdef ENHANCED_BD
                //xENETRxDescriptors[ uxNextRxBuffer ].timestamp;
		//xENETRxDescriptors[ uxNextRxBuffer ].length_proto_type;
		//xENETRxDescriptors[ uxNextRxBuffer ].payload_checksum;
                #endif
                #endif
                //not required by uIP
                #ifdef ENHANCED_BD
                //xENETRxDescriptors[ uxNextRxBuffer ].ebd_status;
                #endif
                
        	/* Doing this here could cause corruption! */		
      		xENETRxDescriptors[ uxNextRxBuffer ].status |= RX_BD_E;
		
		portENTER_CRITICAL();
		{                  
                        uxNextRxBuffer++;
			if( uxNextRxBuffer >= configNUM_ENET_RX_BUFFERS )
			{
				uxNextRxBuffer = 0;
			}
		}
		portEXIT_CRITICAL();	

		/* Make sure the Tx is not using the next buffer before letting the DMA
		know the buffer is available. */
		while( xENETRxDescriptors[ uxNextRxBuffer ].data == pxENETTxDescriptor->data )
		{
			vTaskDelay( enetMINIMAL_DELAY );
		}

		ENET_RDAR = ENET_RDAR_RDAR_MASK;		
	}

    return ulLen;
}
/*-----------------------------------------------------------*/

void vENETTx( void )
{
	/* When we get here the Tx descriptor should show as having completed. */
	while( pxENETTxDescriptor->status & TX_BD_R )
	{
		vTaskDelay( enetMINIMAL_DELAY );
	}

	portENTER_CRITICAL();
	{
                #ifdef NBUF_LITTLE_ENDIAN
                /* To maintain the zero copy implementation, point the Tx descriptor
		to the data from the Rx buffer. */
		pxENETTxDescriptor->data = (uint8_t *)__REV((uint32_t)uip_buf);		
		/* Setup the buffer descriptor for transmission */
		pxENETTxDescriptor->length = __REVSH(uip_len);
                #else
                /* To maintain the zero copy implementation, point the Tx descriptor
		to the data from the Rx buffer. */
		pxENETTxDescriptor->data = uip_buf;
		/* Setup the buffer descriptor for transmission */
		pxENETTxDescriptor->length = uip_len;
                #endif
		#ifdef ENHANCED_BD
          	pxENETTxDescriptor->bdu = 0x00000000;
		pxENETTxDescriptor->ebd_status = TX_BD_INT | TX_BD_TS;// | TX_BD_IINS | TX_BD_PINS;
                #endif
		
		/* NB this assumes only one Tx descriptor! */
		pxENETTxDescriptor->status = ( TX_BD_R | TX_BD_L | TX_BD_TC | TX_BD_W );
	}
	portEXIT_CRITICAL();

	/* Continue the Tx DMA task (in case it was waiting for a new TxBD) */
	ENET_TDAR = ENET_TDAR_TDAR_MASK;
}
/*-----------------------------------------------------------*/

void vENETISRHandler( void )
{
unsigned portLONG ulEvent;
portBASE_TYPE xHighPriorityTaskWoken = pdFALSE;
   
	/* Determine the cause of the interrupt. */
	ulEvent = ENET_EIR & ENET_EIMR;
	ENET_EIR = ulEvent;

	if( ulEvent & ENET_EIR_RXF_MASK )
	{
		/* A packet has been received.  Wake the handler task in case it is 
		blocked. */
		xSemaphoreGiveFromISR( xENETSemaphore, &xHighPriorityTaskWoken );
	}
	
	if( ulEvent & ENET_EIR_TXF_MASK )
	{
		/* The Tx has completed.  Mark the data pointer as NULL so the uIP
		task knows the Rx buffer to which it was pointing is now free again. */
		pxENETTxDescriptor->data = NULL;
	}

	if (ulEvent & ( ENET_EIR_UN_MASK | ENET_EIR_RL_MASK | ENET_EIR_LC_MASK | ENET_EIR_EBERR_MASK | ENET_EIR_BABT_MASK | ENET_EIR_BABR_MASK | ENET_EIR_EBERR_MASK ) )
	{
		/* Sledge hammer error handling. */
		prvInitialiseENETBuffers();
		ENET_RDAR = ENET_RDAR_RDAR_MASK;
	}

	portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
}
