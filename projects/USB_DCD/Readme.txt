   ********************************************************************************
		Release notes for USB DCD demo for Kinetis family
 
			Freescale Semiconductor November 2010
   ********************************************************************************

DCD demo code is an integration of the DCD and the USB-device application that basically 
shows in the terminal what type of HOST controller is connected to the USB module (working
in device mode)
Demo code runs on the Tower system K40 and K60 modules and a TWR-serial module is needed
because the USB port of Kinetis is mapped to the USB port of the TWR-serial board. The
elevator modules are needed as well

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
The DCD example code sends a message to a terminal showing what type of host is
attached to the USB module. Also USB will recognize the board as a serial interface
(COM port). The driver for this is inicluded in the folder where the source files
are stored.
The terminal demo is a basic echo that returns the same character that was sent.