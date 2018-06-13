#ifndef __usb_lock_h__
#define __usb_lock_h__
#include "psptypes.h"
extern volatile uint_8 usb_int_disable;
extern volatile uint_8 usb_int_en_copy;
extern void usb_int_dis(void);
extern void usb_int_en(void);

#endif

