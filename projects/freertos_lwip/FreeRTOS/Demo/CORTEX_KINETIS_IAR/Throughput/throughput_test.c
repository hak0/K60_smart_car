/*
 * Throughput test for UDP/TCP in RX/TX mode. Receiver gets Bytes/sec
 */

/* ------------------------ FreeRTOS includes ----------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* ------------------------ lwIP includes --------------------------------- */
#include "lwip/api.h"
#include "lwip/tcpip.h"
#include "lwip/ip.h"
#include "lwip/memp.h"
#include "lwip/stats.h"
#include "netif/loopif.h"

#include "tcp.h"
#include "udp.h"

#include <string.h>

#include "common.h"
#include "throughput_test.h"

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*
 * UDP/TCP Port used for benchmark
 */
#define  BENCHMARK_PORT                      5678
/*
 * Packet size to send (TX)
 */
#define  BENCHMARK_PACKET_LENGTH             2000/*bytes*/

/*
 * Packet size to send (TX)
 */
#define  BENCHMARK_NUMBER_OF_PACKETS         10000

/*
* 0: means RX (MCU Server - PC Client: Connect/Send)
* 1: means TX (MCU Client - PC Server: Listen/Receive)
 */
#define  BENCHMARK_SEND_TO_PC                1
/*
 * 0: TCP
 * 1: UDP
 */
#define  BENCHMARK_PROTOCOL                  1

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*------------------------------------------------------------------------------*/
/*                            GLOBALS                                          */
/*------------------------------------------------------------------------------*/
static struct netif ENET_if;

#if BENCHMARK_SEND_TO_PC
static int8 array[BENCHMARK_PACKET_LENGTH];
#endif

/*FSL extern function*/
extern int periph_clk_khz;

uint8  first_counter=0;
uint32 total_length_received;
uint32 load_value, current_timer, time_sfw;

uint32 pkt_counter;

/*****************************Prototypes**************************************/


/*****************************Functions***************************************/

/*-----------------------------------------------------------------------------------*/
void 
udp_receive_fnc(void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
      /*FLS: unused arguments*/
      (void)arg;
      (void)pcb;
      (void)addr;
      (void)port;
  
      if (p != NULL) 
      {
           //printf("+");
           /*FSL: select between start, stop or get final results*/
           if( p->tot_len == 2 )//start timer
           {
             first_counter = 0;//start/start again
             total_length_received = 0;
             PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;//enable timer
           }
           else if( p->tot_len == 1 )//stop timer
           {
             if(!first_counter)
             {
               double final_result;
               
               first_counter = 1;//stop count
               
               /*calculate*/
               time_sfw = (load_value + 1 - current_timer);
               PIT_TCTRL0 = 0;//stop the timer
        
               printf("Benchmark results for UDP Rx:\n\n");
               printf("Number of bytes received = %d\n",total_length_received);
               printf("Number of ticks = %d\n",time_sfw);
               printf("Frequency of ticks = %d\n",(configCPU_CLOCK_HZ/2));
               printf("Calculate benchmark: Bytes/sec\n\n");
               
               final_result = (double)total_length_received * (configCPU_CLOCK_HZ/2) /(double)time_sfw;
               printf("Measured throughput is %9.2f Bytes/sec\n",final_result);
             }
           }
           else
           {
             current_timer = PIT_CVAL0;//get last time
             total_length_received += p->tot_len;
           }           

#if 0//no need to send it back!
           udp_sendto(pcb, p, IP_ADDR_BROADCAST, BENCHMARK_PORT);
#endif            
           pbuf_free(p);
      }
}

/*-----------------------------------------------------------------------------------*/
/**
 * Ethernet to PERF task
 *
 * @param none  
 * @return none
 */
void
BRIDGE_PERF_Task( void *pvParameters )
{   
    struct ip_addr  xIpAddr, xNetMast, xGateway;
    extern err_t ethernetif_init( struct netif *netif );

    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;//enable PIT
    
    /* Parameters are not used - suppress compiler error */
    ( void )pvParameters;
    
    tcpip_init( NULL, NULL );

    /* Create and configure the FEC interface. */
    IP4_ADDR( &xIpAddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3 );
    IP4_ADDR( &xNetMast, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3 );
    IP4_ADDR( &xGateway, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3 );
    netif_add( &ENET_if, &xIpAddr, &xNetMast, &xGateway, NULL, ethernetif_init, tcpip_input );    
    
    /* make it the default interface */
    netif_set_default( &ENET_if );

    /* bring it up */
    netif_set_up( &ENET_if );

#if BENCHMARK_PROTOCOL
//**********************************UDP**********************************//
    {
     struct udp_pcb *remote_pcb;
      
     remote_pcb = udp_new();
    
     if(!remote_pcb)
         vTaskDelete(NULL);  /*FSL: error during buffer creation*/

#if BENCHMARK_SEND_TO_PC
//**********************************TX***********************************//
     {  
       uint16 i;
       /*client mode: connect to this server address*/
       struct ip_addr ServerIPAddr;
       struct pbuf *p; 
       
       /*Fill the array*/
       for(i=0;i<sizeof(array);i++)
         array[i] = i%256;//dummy sequence
       
       /*Server IP address to connect*/
       IP4_ADDR(&ServerIPAddr,192,168,0,1);

       printf("Starting UDP TX Client\n");
    
       if( udp_connect(remote_pcb, &ServerIPAddr, (u16_t)BENCHMARK_PORT) != ERR_OK )
         vTaskDelete(NULL);  /*FSL: error during socket creation*/
    
       if( (p = pbuf_alloc(PBUF_TRANSPORT,0,PBUF_REF)) == NULL )
         vTaskDelete(NULL);  /*FSL: error during buffer creation*/
    
       p->payload = &array[0];
       p->len = p->tot_len = BENCHMARK_PACKET_LENGTH;
    
       /*FSL: send selected number of packets*/
       for(pkt_counter=0;pkt_counter<BENCHMARK_NUMBER_OF_PACKETS;pkt_counter++)
       {
          if( udp_send(remote_pcb, p) != ERR_OK )
            break;//error: abort
       }
    
       /*FSL: send ending frames*/
       p->len = p->tot_len = 1;
       /*FSL: send 10 times*/
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
       udp_send(remote_pcb, p);
    
       //FSL: remove connection
       pbuf_free(p);
       udp_remove(remote_pcb);
    
       printf("Leaving UDP Tx Benchmark Test\n");
       vTaskDelete(NULL);  /*FSL: task no longer needed*/
     }
#else
//**********************************RX***********************************//
     {
       printf("Starting UDP RX Server\n");
       
       PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;//enable timer
       PIT_MCR &= ~PIT_MCR_MDIS_MASK;
       PIT_LDVAL0 = 0xFFFFFFFF;//max value
       load_value = PIT_LDVAL0;
       PIT_TCTRL0 = 0;
    
       if( udp_bind(remote_pcb, IP_ADDR_ANY, BENCHMARK_PORT) != ERR_OK )
         printf("Wrong bind to UDP port\n");
       udp_recv(remote_pcb, udp_receive_fnc, NULL);
       /*Task no longer needed*/
       vTaskDelete(NULL);
     }
#endif
    } 
#else
//**********************************TCP**********************************//
    
#if BENCHMARK_SEND_TO_PC
//**********************************TX***********************************//
    {
      struct netconn *conn;
      uint16 i;
    
      /*client mode: connect to this server address*/
      struct ip_addr ServerIPAddr;
      
      /*Fill the array*/
      for(i=0;i<sizeof(array);i++)
         array[i] = i%256;//dummy sequence
      
      /*Server IP address to connect*/
      IP4_ADDR(&ServerIPAddr,192,168,0,1);
      
      printf("Starting TCP Tx Benchmark Test\n");

      /*FSL: wait forever until getting a connection to a valid server*/
      do
      {
        printf("Trying to connect to server...\n");
        
        /* Create a new TCP PCB. */
        conn = netconn_new(NETCONN_TCP);

        /*client connection*/
        if( netconn_connect(conn, &ServerIPAddr, (uint16)BENCHMARK_PORT) != ERR_OK )
        {
          /*close connection for a new SYN process*/
          netconn_close(conn);
          netconn_delete(conn);
          vTaskDelay(2000);
        }
        else
          break;
      }
      while(1);/*infinite loop until getting a valid SYN/SYN-ACK/ACK*/

      printf("Connected to server\n");
      
      /*FSL: send selected number of packets*/
      pkt_counter = BENCHMARK_NUMBER_OF_PACKETS;//will send in a moment
      
      /*FSL: only send when connection is established, otherwise close*/
      while( pkt_counter-- )
      {
        netconn_write(conn, &array[0], sizeof(array), NETCONN_NOCOPY);
      }
      
      /*FSL: no more packets*/
      netconn_close(conn);
      netconn_delete(conn);
      
      printf("Leaving TCP Tx Benchmark Test\n");
      
      /*FSL: good bye*/
      vTaskDelete(NULL);
    }
#else
//**********************************RX***********************************//
  for(;;)
  {
    struct netconn *conn, *newconn;
    struct netbuf *inbuf;
    uint32 total_length_received = 0;
    double final_result;
      
    /* Create a new TCP PCB. */
    conn = netconn_new(NETCONN_TCP);
      
    if( netconn_bind(conn,IP_ADDR_ANY,(uint16)BENCHMARK_PORT) != ERR_OK )
    {
        /*close connection for a new SYN process*/
        netconn_close(conn);
        netconn_delete(conn);
        /*FSL: good bye*/
        vTaskDelete(NULL);
    }
        
      printf("Starting TCP Rx Benchmark Test\n");
      
      /* Put the connection into LISTEN state. */
      netconn_listen(conn);
      
      PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;//enable timer
      PIT_MCR &= ~PIT_MCR_MDIS_MASK;
      PIT_LDVAL0 = 0xFFFFFFFF;//max value
      load_value = PIT_LDVAL0;
      PIT_TCTRL0 = 0;
      
      /* waiting for connection from client */
      newconn = netconn_accept(conn);
      
      PIT_TCTRL0 = PIT_TCTRL_TEN_MASK;//enable timer
      
      for(;;)
      {
        if( (inbuf = netconn_recv(newconn)) != NULL )
        {
           /*data is not touch: inbuf->ptr and inbuf->ptr->tot_len*/
           total_length_received += inbuf->ptr->tot_len;
           /*free pbuf memory*/
           netbuf_delete(inbuf);
        }
        else
        {
           current_timer = PIT_CVAL0;//get last time
           break;/*connection closed*/
        }
      }

      /*FSL: no more packets*/
      netconn_close(newconn);
      netconn_delete(newconn);
      
      netconn_close(conn);
      netconn_delete(conn);
      
      /*calculate*/
      time_sfw = (load_value + 1 - current_timer);
      PIT_TCTRL0 = 0;//stop the timer
        
      printf("Benchmark results for TCP Rx:\n\n");
      printf("Number of bytes received = %d\n",total_length_received);
      printf("Number of ticks = %d\n",time_sfw);
      printf("Frequency of ticks = %d\n",(configCPU_CLOCK_HZ/2));
      printf("Calculate benchmark: Bytes/sec\n\n");
      
      final_result = (double)total_length_received * (configCPU_CLOCK_HZ/2) /(double)time_sfw;
      printf("Measured throughput is %9.2f Bytes/sec\n",final_result);
  }
#endif
#endif
}