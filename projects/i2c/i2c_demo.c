/*
 * File:        i2c_demo.c
 * Purpose:     I2C example of talking to MMA7660 on the Tower boards.
 *
 *              This demo uses a single byte read, but there is also a
 *                u8MMA7660ReadThreeRegisters() function available to show
 *                an example of doing multiple reads back to back.
 *
 *              TWR-K40X256 uses I2C1
 *              TWR-K60N512 uses I2C0
 *              TWR-K53N512 uses I2C1
 *
 */

#include "common.h"
#include "lptmr.h"

//Determine which chip is being used to determine if need to configure I2C0 or I2C1
#ifdef TWR_K60N512
  #include "k60_i2c.h"
#elif TWR_K40X256
  #include "k40_i2c.h"
#else
  #include "k53_i2C.h"
#endif

//Function declarations
void delay(void);
signed char convert (signed char input);
/********************************************************************/
/*

 */
void main (void)
{
  signed char resultx, resulty, resultz;

  printf("Kinetis I2C Demo\n");

  //Initialize I2C
  init_I2C();

  //Configure MMA7660 sensor
  MMA7660WriteRegister(0x09,0xE0); //Disable tap detection
  MMA7660WriteRegister(0x07,0x19); //Enable auto-sleep, auto-wake, and put in active mode

  printf("  X     Y     Z\n");

  while(1)
  {
    //Read x-axis register
    resultx = u8MMA7660ReadRegister(0x00);
    printf("%3d", convert(resultx));

    //Read y-axis register
    resulty = u8MMA7660ReadRegister(0x01);
    printf("   %3d", convert(resulty));

    //Read z-axis register
    resultz = u8MMA7660ReadRegister(0x02);
    printf("   %3d\n", convert(resultz));

    //Delay for 250ms
    time_delay_ms(250);
  }

}

/*******************************************************************/
/*
 * Convert 6-bit accelerometer result into an 8-bit signed char
 */
signed char convert(signed char input)
{
      input &=~0x40; //Turn off alert bit
      if(input & 0x20)    // update 6-bit signed format to 8-bit signed format
	input |= 0xE0;
    return (signed char) input;
}
