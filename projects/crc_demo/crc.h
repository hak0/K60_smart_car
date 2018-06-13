#ifndef __CRC_H
#define __CRC_H

// Type definitions
typedef volatile uint16_t vuint16_t;
typedef volatile uint32_t vuint32_t;
typedef volatile uint8_t  vuint8_t;

// Definitions
#define CRC_BASEADDR  0x40032000  // CRC Starting Address

#define CRC_CRC		(*(vuint32_t*)(CRC_BASEADDR + 0x0000))
#define CRC_CRC_HIGH   	(*(vuint16_t*)(CRC_BASEADDR + 0x0002))  // high word
#define CRC_CRC_LOW    	(*(vuint16_t*)(CRC_BASEADDR + 0x0000))  // low word
#define CRC_CRC_HU   	(*(vuint8_t*)(CRC_BASEADDR + 0x0003))   // high word upper byte
#define CRC_CRC_HL    	(*(vuint8_t*)(CRC_BASEADDR + 0x0002))   // high word low byte
#define CRC_CRC_LU   	(*(vuint8_t*)(CRC_BASEADDR + 0x0001))   // low word upper byte
#define CRC_CRC_LL    	(*(vuint8_t*)(CRC_BASEADDR + 0x0000))   // low word low byte

#define CRC_GPOLY	(*(vuint32_t*)(CRC_BASEADDR + 0x0004))
#define CRC_CTRL	(*(vuint32_t*)(CRC_BASEADDR + 0x0008))

// Error code definition
#define CRC_ERR_SUCCESS       0  
#define CRC_ERR_CODE_BASE    (0x2000)
#define CRC_ERR_TOT_VAL      (CRC_ERR_CODE_BASE+1)
#define CRC_ERR_TOTR_VAL     (CRC_ERR_CODE_BASE+2)
#define CRC_ERR_FXOR_VAL     (CRC_ERR_CODE_BASE+3)
#define CRC_ERR_TCRC_VAL     (CRC_ERR_CODE_BASE+4)

// Prototypes
int CRC_Config(uint32_t poly,uint32_t tot,uint32_t totr,uint32_t fxor,uint32_t tcrc);
uint32_t CRC_Cal_16(uint32_t seed,uint8_t *msg, uint32_t sizeBytes);
uint32_t CRC_Cal_32(uint32_t seed,uint8_t *msg, uint32_t sizeBytes);
#endif