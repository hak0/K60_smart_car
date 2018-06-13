   ********************************************************************************
		Release notes for USB device demo for Kinetis family
 
			Freescale Semiconductor November 2010
   ********************************************************************************

USB Device example emulates a COM port in a Windows HOST and creates an echo aplication.
This means that everything that goes out from the terminal returns to the terminal


===========
Basic Setup
===========

TWR-serial setup
----------------
J16	
1-2	-
3-4	ON
5-6	-


================
Demo explanation
================
Download the code and run it.
Windows PC will request a driver (included in the source files folder). After that just 
start a hyperterminal opening the COM created by the MCU and start typing. all characters
comes back immediately