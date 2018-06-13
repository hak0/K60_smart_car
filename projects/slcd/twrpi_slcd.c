/*
 * File:        twrpi_slcd.c
 * Purpose:     Driver for Segment LCD Tower Plugin (TWRPI-SLCD)
 *              Initializes LCD and control each segment
 *
 */

#include "common.h"
#include "mcg.h"
#include "twrpi_slcd.h"

void init_twrpi_lcd()
{
  // enable IRCLK
  MCG_C1|= 0x02;

  //enable LCD clocks
  SIM_SCGC3 = 0x40000000;

  //enable LCD pins 0, 1, 2, 3, 12, 13 14 15 20 21 22
  LCD_PENL= 0x0070F00F;

  //enable LCD backplanes 0, 1, 2, 3
  LCD_BPENL = 0x0000000F;

  //assign back planes  - 0 is BP1, 1 is BP2 etc
  LCD_WF3TO0 = 0x08040201;

  //Turn on all segments
  tlcd_all_on();

  //enable VIREG and Set TRIM
  //LCD_GCR|= LCD_GCR_RVEN_MASK;
  //LCD_GCR|= 0x0B000000;

  LCD_GCR|=LCD_GCR_VSUPPLY(1);
  LCD_GCR|=LCD_GCR_CPSEL_MASK;
  LCD_GCR|=LCD_GCR_DUTY(3);
  LCD_GCR|=LCD_GCR_SOURCE_MASK; //select alt clock IRC
  LCD_GCR|=LCD_GCR_LCLK(4);

  //Enable LCD
  LCD_GCR|= LCD_GCR_LCDEN_MASK;
}

void tlcd_all_on()
{

          LCD_WF15TO12 = 0x0f0f0f0f;
          LCD_WF23TO20 = 0x000f0f0f;
}

void tlcd_all_off()
{

          LCD_WF15TO12 = 0x00000000;
          LCD_WF23TO20 = 0x00000000;
}

void tlcd_every_other_on()
{

          LCD_WF15TO12 = 0x05050505;
          LCD_WF23TO20 = 0x00050505;
}

void tlcd_degrees_on()
{

          LCD_WF23TO20 |= 0x00000100;

}
void tlcd_degrees_off()
{

          LCD_WF23TO20 &= 0xfffffeff;

}


void tlcd_percent_on()
{

          LCD_WF23TO20 |= 0x00000200;

}
void tlcd_percent_off()
{

          LCD_WF23TO20 &= 0xfffffDff;

}

void tlcd_am_on()
{

          LCD_WF23TO20 |= 0x00000400;

}
void tlcd_am_off()
{

          LCD_WF23TO20 &= 0xfffffBff;

}

void tlcd_pm_on()
{

          LCD_WF23TO20 |= 0x00000800;

}
void tlcd_pm_off()
{

          LCD_WF23TO20 &= 0xfffff7ff;

}

void tlcd_1_on()
{

          LCD_WF23TO20 |= 0x00080000;

}
void tlcd_1_off()
{

          LCD_WF23TO20 &= 0xfff7ffff;

}


void tlcd_fsl_on()
{

          LCD_WF15TO12 |= 0x08000000;

}
void tlcd_fsl_off()
{

          LCD_WF15TO12 &= 0xf7ffffff;

}


void tlcd_colon_on()
{

          LCD_WF15TO12 |= 0x00000800;

}
void tlcd_colon_off()
{

          LCD_WF15TO12 &= 0xfffff7ff;

}

void tlcd_set_numeric1(uint32 n)
{
  switch(n)
  {
  case 0:
    {
          LCD_WF15TO12 |= 0x050f0000;
          LCD_WF15TO12 &= 0xfDffffff;
    break;}
  case 1:
    {
          LCD_WF15TO12 |= 0x00060000;
          LCD_WF15TO12 &= 0xf8f6ffff;
    break;}
  case 2:
    {
          LCD_WF15TO12 |= 0x060B0000;
          LCD_WF15TO12 &= 0xfEfBffff;
    break;}
  case 3:
    {
          LCD_WF15TO12 |= 0x020F0000;
          LCD_WF15TO12 &= 0xfAfFffff;
    break;}
  case 4:
    {
          LCD_WF15TO12 |= 0x03060000;
          LCD_WF15TO12 &= 0xfBf6ffff;
    break;}
  case 5:
    {
          LCD_WF15TO12 |= 0x030D0000;
          LCD_WF15TO12 &= 0xfBfDffff;
    break;}
  case 6:
    {
          LCD_WF15TO12 |= 0x070D0000;
          LCD_WF15TO12 &= 0xfFfDffff;
    break;}
  case 7:
    {
          LCD_WF15TO12 |= 0x00070000;
          LCD_WF15TO12 &= 0xf8f7ffff;
    break;}
   case 8:
    {
          LCD_WF15TO12 |= 0x070F0000;
          LCD_WF15TO12 &= 0xfFfFffff;
    break;}

   case 9:
    {
          LCD_WF15TO12 |= 0x030F0000;
          LCD_WF15TO12 &= 0xfBfFffff;
          break;
    }

    default:

      break;
}
}

void tlcd_set_numeric2(uint32 n)
{
  switch(n)
  {
  case 0:
    {
          LCD_WF15TO12 |= 0x0000050f;
          LCD_WF15TO12 &= 0xfffffDff;
    break;}
  case 1:
    {
          LCD_WF15TO12 |= 0x00000006;
          LCD_WF15TO12 &= 0xfffff8f6;
    break;}
  case 2:
    {
          LCD_WF15TO12 |= 0x0000060B;
          LCD_WF15TO12 &= 0xfFfFfEfB;
    break;}
  case 3:
    {
          LCD_WF15TO12 |= 0x0000020F;
          LCD_WF15TO12 &= 0xfffFfAff;
    break;}
  case 4:
    {
          LCD_WF15TO12 |= 0x00000306;
          LCD_WF15TO12 &= 0xfffffbf6;
    break;}
  case 5:
    {
          LCD_WF15TO12 |= 0x0000030d;
          LCD_WF15TO12 &= 0xfffffbfd;
    break;}
  case 6:
    {
          LCD_WF15TO12 |= 0x0000070D;
          LCD_WF15TO12 &= 0xfFfffffd;
    break;}
  case 7:
    {
          LCD_WF15TO12 |= 0x00000007;
          LCD_WF15TO12 &= 0xfFfff8f7;
    break;}
   case 8:
    {
          LCD_WF15TO12 |= 0x0000070F;
          LCD_WF15TO12 &= 0xfFfFffff;
    break;}

   case 9:
    {
          LCD_WF15TO12 |= 0x0000030F;
          LCD_WF15TO12 &= 0xfffFfBff;
          break;
    }

    default:

      break;
}
}

void tlcd_set_numeric3(uint32 n)
{

  switch(n)
  {
  case 0:
    {
          LCD_WF23TO20 |= 0x0005000f;
          LCD_WF23TO20 &= 0xfffDffff;
    break;}
  case 1:
    {
          LCD_WF23TO20 |= 0x00000006;
          LCD_WF23TO20 &= 0xfff8fff6;
    break;}
  case 2:
    {
          LCD_WF23TO20 |= 0x0006000B;
          LCD_WF23TO20 &= 0xfFfEfffB;
    break;}
  case 3:
    {
          LCD_WF23TO20 |= 0x0002000F;
          LCD_WF23TO20 &= 0xfffAffff;
    break;}
  case 4:
    {
          LCD_WF23TO20 |= 0x00030006;
          LCD_WF23TO20 &= 0xfffbfff6;
    break;}
  case 5:
    {
          LCD_WF23TO20 |= 0x0003000d;
          LCD_WF23TO20 &= 0xfffbfffd;
    break;}
  case 6:
    {
          LCD_WF23TO20 |= 0x0007000D;
          LCD_WF23TO20 &= 0xfFfffffd;
    break;}
  case 7:
    {
          LCD_WF23TO20 |= 0x00000007;
          LCD_WF23TO20 &= 0xfFf8fff7;
    break;}
   case 8:
    {
          LCD_WF23TO20 |= 0x0007000F;
          LCD_WF23TO20 &= 0xfFfFffff;
    break;}

   case 9:
    {
          LCD_WF23TO20 |= 0x0003000F;
          LCD_WF23TO20 &= 0xfffBffff;
          break;
    }

    default:

      break;
}
}