/*
 * File:        i2c.c
 * Purpose:     Code for initializing and using I2C
 *
 * Notes:
 *
 */

#include "common.h"
#include "i2c.h"


//unsigned char MasterTransmission;
//unsigned char SlaveID;


/*******************************************************************/
/*!
 * I2C Initialization
 * Set Baud Rate and turn on I2C
 */
void init_I2C(void)
{
    SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;

    /* configure GPIO for I2C function */
    PORTC_PCR11 = PORT_PCR_MUX(2);
    PORTC_PCR10 = PORT_PCR_MUX(2);

    I2C1_F  = 0x14;       /* set MULT and ICR */

    I2C1_C1 = 0x80;       /* enable IIC */
}

/*******************************************************************/
/*!
 * Pause Routine
 */
void Pause(void){
    int n;
    for(n=1;n<50;n++) {
      asm("nop");
    }
}

/*******************************************************************/
/*!
 * Start I2C Transmision
 * @param SlaveID is the 7 bit Slave Address
 * @param Mode sets Read or Write Mode
 */
void IIC_StartTransmission (unsigned char SlaveID, unsigned char Mode)
{
  /* shift ID in right possition */
  SlaveID = (unsigned char) SlaveID << 1;

  /* Set R/W bit at end of Slave Address */
  SlaveID |= (unsigned char)Mode;

  /* send start signal */
  i2c_Start();

  /* send ID with W/R bit */
  i2c_write_byte(SlaveID);
}


/*******************************************************************/
/*!
 * Read a register from the MMA7660
 * @param u8RegisterAddress is Register Address
 * @return Data stored in Register
 */
unsigned char I2C_ReadRegister(unsigned char u8I2CSlaveAddress,unsigned char u8RegisterAddress)
{
  unsigned char result;
  unsigned int j;

  /* Send Slave Address */
  IIC_StartTransmission(u8I2CSlaveAddress,MWSR);
  i2c_Wait();

  /* Write Register Address */
  I2C1_D = u8RegisterAddress;
  i2c_Wait();

  /* Do a repeated start */
  I2C1_C1 |= I2C_C1_RSTA_MASK;

  /* Send Slave Address */
  I2C1_D = (u8I2CSlaveAddress << 1) | 0x01; //read address
  i2c_Wait();

  /* Put in Rx Mode */
  I2C1_C1 &= (~I2C_C1_TX_MASK);

  /* Turn off ACK */
  I2C1_C1 |= I2C_C1_TXAK_MASK;

  /* Dummy read */
  result = I2C1_D ;
  for (j=0; j<5000; j++){};
  i2c_Wait();

  /* Send stop */
  i2c_Stop();
  result = I2C1_D ;
  Pause();
  return result;
}

/*******************************************************************/
/*!
 * Write a byte of Data to the MMA7660
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void I2C_WriteRegister(unsigned char u8I2CSlaveAddress,unsigned char u8RegisterAddress, unsigned char u8Data)
{
  /* send data to slave */
  IIC_StartTransmission(u8I2CSlaveAddress,MWSR);
  i2c_Wait();

  /* Send I2C address */
  I2C1_D = u8RegisterAddress;
  i2c_Wait();

  /* Send data */
  I2C1_D = u8Data;
  i2c_Wait();

  i2c_Stop();

  Pause();
}