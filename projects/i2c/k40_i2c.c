/*
 * File:        k40_i2c.c
 * Purpose:     Code for initializing and using I2C
 *
 * Notes:
 *
 */

#include "common.h"
#include "k40_i2c.h"


unsigned char MasterTransmission;
unsigned char SlaveID;


/*******************************************************************/
/*!
 * I2C Initialization
 * Set Baud Rate and turn on I2C
 */
void init_I2C(void)
{
    SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK; //Turn on clock to I2C1 module

    /* Configure GPIO for I2C1 function */
    PORTC_PCR11 = PORT_PCR_MUX(2);
    PORTC_PCR10 = PORT_PCR_MUX(2);

    I2C1_F  = 0x14;       /* set MULT and ICR */

    I2C1_C1 = I2C_C1_IICEN_MASK;       /* enable IIC */
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
  if(Mode == MWSR)
  {
    /* set transmission mode */
    MasterTransmission = MWSR;
  }
  else
  {
    /* set transmission mode */
    MasterTransmission = MRSW;
  }

  /* shift ID in right possition */
  SlaveID = (unsigned char) MMA7660_I2C_ADDRESS << 1;

  /* Set R/W bit at end of Slave Address */
  SlaveID |= (unsigned char)MasterTransmission;

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
unsigned char u8MMA7660ReadRegister(unsigned char u8RegisterAddress)
{
  unsigned char result;

  /* Send Slave Address */
  IIC_StartTransmission(SlaveID,MWSR);
  i2c_Wait();

  /* Write Register Address */
  I2C1_D = u8RegisterAddress;
  i2c_Wait();

  /* Do a repeated start */
  I2C1_C1 |= I2C_C1_RSTA_MASK;

  /* Send Slave Address */
  I2C1_D = (MMA7660_I2C_ADDRESS << 1) | 0x01; //read address
  i2c_Wait();

  /* Put in Rx Mode */
  I2C1_C1 &= (~I2C_C1_TX_MASK);

  /* Turn off ACK */
  I2C1_C1 |= I2C_C1_TXAK_MASK;

  /* Dummy read */
  result = I2C1_D ;
  i2c_Wait();

  /* Send stop */
  i2c_Stop();
  result = I2C1_D ;
  return result;
}

/*******************************************************************/
/*!
 * Read first three registers from the MMA7660
 * @param u8RegisterAddress is Register Address
 * @return Data stored in Register
 */
unsigned char u8MMA7660ReadThreeRegisters(unsigned char u8RegisterAddress)
{
  unsigned char result1, result2, result3;

  /* Send Slave Address */
  IIC_StartTransmission(SlaveID,MWSR);
  i2c_Wait();

  /* Write Register Address */
  I2C1_D = u8RegisterAddress;
  i2c_Wait();

  /* Do a repeated start */
  I2C1_C1 |= I2C_C1_RSTA_MASK;

  /* Send Slave Address */
  I2C1_D = (MMA7660_I2C_ADDRESS << 1) | 0x01; //read address
  i2c_Wait();

  /* Put in Rx Mode */
  I2C1_C1 &= (~I2C_C1_TX_MASK);

  /* Ensure TXAK bit is 0 */
  I2C1_C1 &= ~I2C_C1_TXAK_MASK;

  /* Dummy read */
  result1 = I2C1_D ;
  i2c_Wait();

  /* Read first byte */
  result1 = I2C1_D;
  i2c_Wait();

  /* Turn off ACK since this is second to last read*/
  I2C1_C1 |= I2C_C1_TXAK_MASK;

  /* Read second byte */
  result2 = I2C1_D;
  i2c_Wait();

  /* Send stop */
  i2c_Stop();

  /* Read third byte */
  result3 = I2C1_D;

  printf("%3d    %3d     %3d\n",result1,result2,result3);
  return result1;
}


/*******************************************************************/
/*!
 * Write a byte of Data to the MMA7660
 * @param u8RegisterAddress is Register Address
 * @param u8Data is Data to write
 */
void MMA7660WriteRegister(unsigned char u8RegisterAddress, unsigned char u8Data)
{
  /* send data to slave */
  IIC_StartTransmission(SlaveID,MWSR);
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