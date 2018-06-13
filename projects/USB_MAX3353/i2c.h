/*
 * File:        i2c.h
 * Purpose:     I2C header
 *
 * Notes:
 *
 */

#define MMA7660_I2C_ADDRESS                         0x4C


#define i2c_DisableAck()       I2C1_C1 |= I2C_C1_TXAK_MASK

#define i2c_RepeatedStart()    I2C1_C1     |= 0x04;

#define i2c_Start()            I2C1_C1     |= 0x10;\
                               I2C1_C1     |= I2C_C1_MST_MASK

#define i2c_Stop()             I2C1_C1  &= ~I2C_C1_MST_MASK;\
                               I2C1_C1  &= ~I2C_C1_TX_MASK

#define i2c_EnterRxMode()      I2C1_C1   &= ~I2C_C1_TX_MASK;\
                               I2C1_C1   &= ~I2C_C1_TXAK_MASK

#define i2c_Wait()               while((I2C1_S & I2C_S_IICIF_MASK)==0) {} \
                                  I2C1_S |= I2C_S_IICIF_MASK;

#define i2c_write_byte(data)   I2C1_D = data

#define MWSR                   0x00  /* Master write  */
#define MRSW                   0x01  /* Master read */

void init_I2C(void);
void IIC_StartTransmission (unsigned char SlaveID, unsigned char Mode);
void I2C_WriteRegister(unsigned char, unsigned char u8RegisterAddress, unsigned char u8Data);
unsigned char I2C_ReadRegister(unsigned char, unsigned char u8RegisterAddress);
