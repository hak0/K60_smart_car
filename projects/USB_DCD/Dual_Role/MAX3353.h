#include "psptypes.h"
#include "i2c.h"
#include "common.h"
#include "Settings.h"

#define MAX3353_I2C_ADDRESS                         0x2C

enum
{
    MAX3353_OK,
    MAX3353_NOT_PRESENT
};

#define MAX3353_MID     0x6A0B5333
#define MAX3353_PID     0x485A4200

// PTB8 INT from MAX3353



#define ID_GND_EVENT            0x20
#define ID_FLOAT_EVENT          0x40
#define VBUS_LOW_EVENT          0x10
#define VBUS_HIGH_EVENT         0x01



#define MAX3353_REG_CTRL_1       0x10
#define MAX3353_REG_CTRL_2       0x11
#define MAX3353_REG_STATUS       0x13
#define MAX3353_REG_INT_MASK     0x14
#define MAX3353_REG_INT_EDGE     0x15
#define MAX3353_REG_INT_LATCH    0x16



#define ID_FLOAT_EN_MASK        0x10
#define ID_GND_EN_MASK          0x08
#define VBUS_VALID_EN_MASK      0x01
#define SESSION_SESSEND_EN_MASK 0x04
#define SESSION_VALID_EN_MASK   0x01

#define VBUS_VALID_ED_MASK      0x01
#define SESSION_VALID_ED_MASK   0x02





/* MAX3353 register bits */
#define OTG_STAT_VBUS_VALID		   0x01
#define OTG_STAT_SESS_VALID		   0x02
#define OTG_STAT_SESS_END		     0x04
#define OTG_STAT_ID_GND			     0x08
#define OTG_STAT_ID_FLOAT		     0x10
#define OTG_STAT_A_HNP			     0x20
#define OTG_STAT_B_HNP			     0x40

#define OTG_INT_VBUS_VALID_CHG   0x01
#define OTG_INT_SESS_VALID_CHG   0x02
#define OTG_INT_SESS_END_CHG     0x10
#define OTG_INT_ID_GND_CHG       0x20
#define OTG_INT_ID_FLOAT_CHG     0x40
#define OTG_INT_A_HNP_CHG        0x80


#define OTG_CTRL_2_SDWN          0x01
#define OTG_CTRL_2_VBUS_CHG1     0x02
#define OTG_CTRL_2_VBUS_CHG2     0x04
#define OTG_CTRL_2_VBUS_DRV      0x08
#define OTG_CTRL_2_VBUS_DISCHG   0x10


#define OTG_CTRL_1_IRQ_PUSH_PULL 0x02
#define OTG_CTRL_1_PDOWN_DP      0x40
#define OTG_CTRL_1_PDOWN_DM      0x80


/* Prototypes */
uint_8 MAX3353_Init(void);
void MAX3353_ISR(void);
uint_8 MAX3353_Read_Register(uint_8);
void MAX3353_Write_Register(uint_8,uint_8);
uint_8 MAX3353_CheckifPresent(void);
