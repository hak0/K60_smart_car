   ********************************************************************************
		Release notes for USB HOST demo for Kinetis family
 
			Freescale Semiconductor November 2010
   ********************************************************************************

Host code demo waits for an HID mouse and basically displays the movement and the button
status of the mouse in the serial console.


===========
Basic Setup
===========

TWR-serial setup
----------------
J16	
1-2	ON
3-4	-
5-6	-


================
Demo explanation
================
Download the code and run it. After the terminal shows the message "waiting for mouse" use
an USB adapter to connect a standard HID mouse th to USB port in the TWR-serial board.
Kinetis will recognize the mouse and will show some information about it and user can
start moving te mouse and pressing the buttons. 