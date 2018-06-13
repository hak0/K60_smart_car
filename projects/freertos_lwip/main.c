/*
    FreeRTOS V6.0.4 - Copyright (C) 2010 Real Time Engineers Ltd.

    ***************************************************************************
    *                                                                         *
    * If you are:                                                             *
    *                                                                         *
    *    + New to FreeRTOS,                                                   *
    *    + Wanting to learn FreeRTOS or multitasking in general quickly       *
    *    + Looking for basic training,                                        *
    *    + Wanting to improve your FreeRTOS skills and productivity           *
    *                                                                         *
    * then take a look at the FreeRTOS eBook                                  *
    *                                                                         *
    *        "Using the FreeRTOS Real Time Kernel - a Practical Guide"        *
    *                  http://www.FreeRTOS.org/Documentation                  *
    *                                                                         *
    * A pdf reference manual is also available.  Both are usually delivered   *
    * to your inbox within 20 minutes to two hours when purchased between 8am *
    * and 8pm GMT (although please allow up to 24 hours in case of            *
    * exceptional circumstances).  Thank you for your support!                *
    *                                                                         *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public 
    License and the FreeRTOS license exception along with FreeRTOS; if not it 
    can be viewed here: http://www.freertos.org/a00114.html and also obtained 
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/

/*
 * Creates webserver or throughput example code
 */

/* Library includes. */
#include "common.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"

/* Apps includes */
#include "HTTPDemo.h"
#include "throughput_test.h"
#include "partest.h"

/*
 * Configure the processor clock and ports.
 */
static void prvSetupHardware( void );

void 
performance_startup(void)
{
//******************************************************************************
// ahb crossbar switch (axbs) configuration registers
// master assignments
//  m0 = processor_code
//  m1 = processor_system
//  m2 = dma/export
//  m3 = enet
//  m4 = usb
//  m5 = esdhc
//
// slave assignments
//  s0 = pflash
//  s1 = tcmc_bkd (backdoor)
//  s2 = aips0
//  s3 = aips1/gpio
//  s4 = flexbus
        
        /* initialize the processor core, crossbar switch and platform flash controller for max performance */

        /* note: certain operations here are simply included as placeholders to reference a config register */

        /* core configuration */
        /* of interest are actlr[1:0] which disable the write buffer [1] and disable interrupts on multi-T insts [0] */
        /* it was discovered that assertion of actlr[0] adds one cycle of execution time to ldm, stm and divides */
        //FLash init:
        SCB_ACTLR = 0;

       /* crossbar configuration */
       /* first, configure the master priorities for fixed arbitration for each slave port: 0 = highest, 7 = lowest */
       /* in general, priority increases as master port number increases, that is, the desired priority is:
          esdhc (m5) > usb (m4) > enet (m3) > dma (m2) > core_sys (m1) > core_code (m0) */
        
        /* crossbar configuration */
        AXBS_PRS0 = 0x76012354;  /* m1 (core's code bus) never accesses pflash */
        AXBS_PRS1 = 0x76012345;  /* m0 (core's sys  bus) never accesses tcmc_bkd */
        AXBS_PRS2 = 0x76012345;  /* m0 (core's sys  bus) never accesses aips0 */
        AXBS_PRS3 = 0x76012345;  /* m0 (core's sys  bus) never accesses aips1/gpio */
        AXBS_PRS4 = 0x76012345;  /* m0 (core's sys  bus) never accesses flexbus */
        
        AXBS_CRS0 = 0x80000000;  /* read-only, fixed pri arb, park pflash     on m0 (core's code bus) */
        AXBS_CRS1 = 0x80000010;  /* read-only, fixed pri arb, park tcmc_bkd   on "last" master */
        AXBS_CRS2 = 0x80000010;  /* read-only, fixed pri arb, park aips0      on "last" master */
        AXBS_CRS3 = 0x80000010;  /* read-only, fixed pri arb, park aips1/gpio on "last" master */
        AXBS_CRS4 = 0x80000010;  /* read-only, fixed pri arb, park flexbus    on "last" master */
        
        AXBS_MGPCR0 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */
        AXBS_MGPCR1 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */
        AXBS_MGPCR2 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */
        AXBS_MGPCR3 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */
        AXBS_MGPCR4 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */
        AXBS_MGPCR5 = 0x2;       /* set aulb = 2 = arbitrate after 4 beats of an unspecified-length incr burst */

        /* platform flash configuration */
        /* the reset state of the pfapr is the preferred configuration, so no update is needed */
      
        /* configure the pflash cache and page buffers */
        FMC_PFB0CR = (3 << 28) /* rwsc (read-only) = 3 cycles for core:flash speed == 100:25 MHz */
                    | (0 << 24) /* clck_way = 0 */
                    | (0 << 20) /* cinv_way = 0 */
                    | (1 << 17) /* b0mw (read-only) = 1 == 64 bit flash page size */
                    | (0 << 5)  /* b0p0crc: 0 = lru on all 4, 2 = lru on if for [0-1], op for [2-3],
                                                              3 = lru on if for [0-2], op for [3] */
                    | (1 << 4)  /* b0p0dce = 1 == data cache enable */
                    | (1 << 3)  /* b0p0dce = 1 == inst cache enable */
                    | (0 << 2)  /* b0p0dpe = 1 == data prefetch enable */
                    | (1 << 1)  /* b0p0ipe = 1 == inst prefetch enable */
                    | (1 << 0); /* b0p0pbe = 1 == page buffer enable */
        
        FMC_PFB1CR = (1 << 4)  /* b1p0dce = 1 == data cache enable */
                    | (1 << 3)  /* b1p0dce = 1 == inst cache enable */
                    | (0 << 2)  /* b1p0dpe = 1 == data prefetch enable */
                    | (1 << 1)  /* b1p0ipe = 1 == inst prefetch enable */
                    | (1 << 0); /* b1p0pbe = 1 == page buffer enable */
}

/*-----------------------------------------------------------*/

/*
 * Starts all the other tasks, then starts the scheduler.
 */
void main( void )
{
	/* Setup any hardware that has not already been configured by the low
	level init routines. */
	prvSetupHardware();

#if 1       
	/* Create the webserver */
	sys_thread_new( ( char * ) "httpd", vBasicWEBServer, ( void * ) NULL, lwipBASIC_SERVER_STACK_SIZE, basicwebWEBSERVER_PRIORITY );
#else
        /* Create the Performance/Throughput App */
	sys_thread_new( ( char * ) "perf", BRIDGE_PERF_Task, ( void * ) NULL, throughput_STACK_SIZE, throughput_PRIORITY );
#endif
        
	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
	for( ;; )
        ;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	performance_startup();
  
        /* Peripheral initialisation. */
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will get called if a task overflows its stack. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
