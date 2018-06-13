#ifndef __USB_DCD__
#define __USB_DCD__

#include "Fsltypes.h"
#include "common.h"
#include "USB.h"



//#define DCD_POLLING_METHOD
#define DCD_INTERRUPT_METHOD

#define DCD_TIME_BASE       48

/* Pin assignment */
#define VBUS_DETECTION      0
#define DCD_ACTIVE          1

#define DCD_SEQ_STAT0       2
#define DCD_SEQ_STAT1       3

#define DCD_SEQ_RES0        4
#define DCD_SEQ_RES1        5

#define DCD_ISR_SIGNAL      6
#define DCD_TO              7
#define DCD_ERR             8
#define DCD_DEBUG           9


#define STANDARD_HOST       1
#define CHARGING_HOST       2
#define DEDICATED_CHARGER   3


#define USBDCD_STATUS_SEQ_STAT0_SHIFT   18
#define USBDCD_STATUS_SEQ_STAT1_SHIFT   19

#define USBDCD_STATUS_SEQ_RES0_SHIFT    16
#define USBDCD_STATUS_SEQ_RES1_SHIFT    17

/* Macros */
#define DCD_Start_Charger_Detection_Sequence_InterruptMode  (USBDCD_CONTROL=(USBDCD_CONTROL_IE_MASK | USBDCD_CONTROL_START_MASK | USBDCD_CONTROL_IACK_MASK))
#define DCD_Start_Charger_Detection_Sequence_PollMode       (USBDCD_CONTROL=(USBDCD_CONTROL_START_MASK | USBDCD_CONTROL_IACK_MASK))
#define DCD_ResetModule                                     (USBDCD_CONTROL=(USBDCD_CONTROL_SR_MASK | USBDCD_CONTROL_IACK_MASK))


/* USBDCD Register Definition */
/*
#define USBDCD_CONTROL                  (*(vUINT32*)(0xFFFF83C0))
#define USBDCD_CONTROL_SR_MASK          0x02000000
#define USBDCD_CONTROL_START_MASK       0x01000000
#define USBDCD_CONTROL_IE_MASK          0x00010000
#define USBDCD_CONTROL_IF_MASK          0x00000100
#define USBDCD_CONTROL_IACK_MASK        0x00000001


#define USBDCD_CLOCK                    (*(vUINT32*)(0xFFFF83C4))

#define USBDCD_STATUS                   (*(vUINT32*)(0xFFFF83C8))
#define USBDCD_STATUS_ACTIVE_MASK       0x00400000        
#define USBDCD_STATUS_TO_MASK           0x00200000
#define USBDCD_STATUS_ERR_MASK          0x00100000        


#define USBDCD_STATUS_SEQ1_STAT_MASK    0x00080000        
#define USBDCD_STATUS_SEQ0_STAT_MASK    0x00040000        
#define USBDCD_STATUS_SEQ1_RES_MASK     0x00020000        
#define USBDCD_STATUS_SEQ0_RES_MASK     0x00010000
#define USBDCD_STATUS_SEQ_RES_MASK      0x00030000
#define USBDCD_STATUS_SEQ_STAT_MASK     0x000C0000



#define USBDCD_TEST             (*(vUINT32*)(0xFFFF83CC))
#define USBDCD_TIMER0           (*(vUINT32*)(0xFFFF83D0))
#define USBDCD_TIMER1           (*(vUINT32*)(0xFFFF83D4))
#define USBDCD_TIMER2           (*(vUINT32*)(0xFFFF83D8))
*/
#define USBDCD_STATUS_FLAGS_MASK        0x00300000


/* USB DCD REgisters reset value */
#define USBDCD_CONTROL_RESET_VALUE  0x00010000
#define USBDCD_CLOCK_RESET_VALUE    0x000000C1
#define USBDCD_STATUS_RESET_VALUE   0x00000000
#define USBDCD_TIMER0_RESET_VALUE   0x00100000
#define USBDCD_TIMER1_RESET_VALUE   0x000A0028
#define USBDCD_TIMER2_RESET_VALUE   0x00280001


/* Prototypes */
void DCD_Init(void);
UINT8 DCD_Service(void);
UINT8 DCD_Out_of_reset_test(void);
UINT8 DCD_RegisterAccess_byte(void);
UINT8 DCD_RegisterAccess_word(void);
UINT8 DCD_RegisterAccess_long(void);
UINT8 DCD_GetChargerType(void);
void  DCD_Signal_Testing(void);
void DCD_ISR(void);





#endif /* __USB_DCD__ */