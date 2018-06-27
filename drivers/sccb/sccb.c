//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///                                  ////
///     感谢野火提供的库文件          ///
///           2015-4-10               ///
//////////////////////////////////////////
#include "sccb.h"
#include "gpio.h"



/*************************************************************************
*                            
*
*  函数名称：sccb_init
*  功能说明：初始化SCCB  其中SCL接PTC6 SDA接PTC7
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
//-------初始化PC6  PC7为GPIO--------//
void sccb_init(void)
{
  PORT_PCR_REG(PORTC_BASE_PTR, 6) = (0 | PORT_PCR_MUX(1) );
  PORT_PCR_REG(PORTC_BASE_PTR, 7) = (0 | PORT_PCR_MUX(1) );
}

/************************************************************************
*                            
*
*  函数名称：sccb_wait
*  功能说明：SCCB延时，不应太小
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void sccb_wait(void)
{
  u32 i;
  for( i=0; i<2000; i++)
  {
    asm ("nop");
  }
//  
//  u8 i;
//  u16 j;
//  for( i=0; i<100; i++)
//  {
//    j++;
//  }

}

/************************************************************************
*                            
*
*  函数名称：sccb_start
*  功能说明：SCCB启动位
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void sccb_start(void)
{
  SCL_OUT;
  SDA_OUT;
 
  SDA_HIGH;
  sccb_wait();
  SCL_HIGH;
  sccb_wait();
  SDA_LOW;
  sccb_wait();
  SCL_LOW;
}

/************************************************************************
*                            
*
*  函数名称：sccb_stop
*  功能说明：SCCB停止位
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void sccb_stop(void)
{
  SCL_OUT;
  SDA_OUT; 
  SDA_LOW;
  sccb_wait();
  SCL_HIGH;
  sccb_wait();
  SDA_HIGH;
  sccb_wait();
}

/************************************************************************
*                             
*
*  函数名称：sccb_sendByte
*  功能说明：在SCCB总线上发送一个字节
*  参数说明：data 要发送的字节内容
*  函数返回：无
*
*************************************************************************/
u8 sccb_sendByte(u8 data)
{
  u8 i;
   u8 ack;
  SDA_OUT;
  for( i=0; i<8; i++)
  {
    if(data & 0x80)
      SDA_HIGH;
    else 
      SDA_LOW;
    data <<= 1;
    sccb_wait();
    SCL_HIGH;
    sccb_wait();
    SCL_LOW;
    sccb_wait();
  }
  SDA_HIGH;
  SDA_IN;
  sccb_wait();
  SCL_HIGH;
  sccb_wait();
  ack = SDA_DATA;
  SCL_LOW;
  sccb_wait();
  return ack;
}


/************************************************************************
*                             
*
*  函数名称：sccb_regWrite
*  功能说明：通过SCCB总线向指定设备的指定地址发送指定内容
*  参数说明：device---设备号  读写有区别
*            address---写数据的寄存器
*            data---写的内容
*  函数返回：ack=1未收到应答(失败)    ack=0收到应答(成功)
*
*************************************************************************/
u8 sccb_regWrite(u8 device,u8 address,u8 data)
{
  u8 i;
  u8 ack;
  for( i=0; i<20; i++)
  {
    sccb_start();
    ack = sccb_sendByte(device);
    if( ack == 1 )
    {
      sccb_stop();
      continue;
    }
    
    ack = sccb_sendByte(address);
    if( ack == 1 )
    {
      sccb_stop();
      continue;
    }
    
    ack = sccb_sendByte(data);
    if( ack == 1 )
    {
      sccb_stop();
      continue;
    }
    
    sccb_stop();
    if( ack == 0 ) break;
  }
  return ack;
}


u8 sccb_refresh()
{
    //-----------SCCB恢复默认出厂设置----------//
    //--PCLK:73ns   HREF:63.6us   VSYN:16.64ms--//
    //--默认隔行扫描，全分辨率为640*480，采集VSYN分辨率是640*240  
    //--在每两个HREF之间有640个PCLK
    //--在每两个VSYN之间有240个HREF    
    //sccb_regWrite(0x42,0x11,0x00);   
    //sccb_regWrite(0x42,0x14,0x04);
    //sccb_regWrite(0x42,0x28,0x20);
    //---------------------------------------//
    u8 ack1,ack2,ack3;
    u8 ACK = 1;
    for(u8 sccb_time=0; sccb_time<10; sccb_time++)
    {
      ack1 = sccb_regWrite(0x42,0x12,0x80);    //COM7 , 复位所有寄存器
      sccb_wait();
      sccb_wait();
      sccb_wait();
      sccb_wait();
      sccb_wait();
//      ack1 = sccb_regWrite(0x42,0x00,0x10);    //AGC 0x03
//      ack1 = sccb_regWrite(0x42,0x09,0x03);    //COM2  0x03
//      ack1 = sccb_regWrite(0x42,0x0c,0x10);    //COM3 
//      ack1 = sccb_regWrite(0x42,0x0d,0x00);    //COM4 0x81
//      ack1 = sccb_regWrite(0x42,0x0e,0x01);    //COM5  
//      
//      ack1 = sccb_regWrite(0x42,0x11,0x80);    //CLKRC 0x80
//      ack1 = sccb_regWrite(0x42,0x12,0x40);    //COM7  QVGA  YUV
//      ack3 = sccb_regWrite(0x42,0x15,0x20);    //COM10  设置PCLK等反转
//      ack1 = sccb_regWrite(0x42,0x17,0x3f);   // HSTART QVGA
//      ack2 = sccb_regWrite(0x42,0x18,0x50);   // HSIZE
//      ack2 = sccb_regWrite(0x42,0x19,0x03);   // VSTRT
//      ack2 = sccb_regWrite(0x42,0x1a,0x78);  //  VSIZE     
//      
//      ack3 = sccb_regWrite(0x42,0x29,0x50);  // HOutSize  50 QVGA  A0 VGA
//      ack3 = sccb_regWrite(0x42,0x2c,0x78);  // VoutSize 78 QVGA  F0 VGA 
//    
//      ack3 = sccb_regWrite(0x42,0x66,0x80);  //  DSP_Ctrl4
//      ack3 = sccb_regWrite(0x42,0x67,0x00);  //  

//#if (CAMERA_W == 80)
//    {HOutSize     ,0x14},
//#elif (CAMERA_W == 160)  
//    {HOutSize     ,0x28},
//#elif (CAMERA_W == 240)  
//    {HOutSize     ,0x3c},
//#elif (CAMERA_W == 320)  
//    {HOutSize     ,0x50}, 
//#else
//    
//#endif
//
//#if (CAMERA_H == 60 )
//    {VOutSize     ,0x1E},
//#elif (CAMERA_H == 120 )
//    {VOutSize     ,0x3c},
//#elif (CAMERA_H == 180 )
//    {VOutSize     ,0x5a},
//#elif (CAMERA_H == 240 )
//    {VOutSize     ,0x78},
//#else
//    
//#endif
     ack1 = sccb_regWrite(0x42,OV7725_COM4,0x41);  //41 4倍分频
     /* ack1 = sccb_regWrite(0x42,OV7725_COM4,0xC1);  //81 8倍分频,噪点严重，放弃 */
     ack1 = sccb_regWrite(0x42,OV7725_CLKRC,0x00); //摄像头时钟以最高速度运行
     //ack1 = sccb_regWrite(0x42,OV7725_COM2,0x03);
     ack1 = sccb_regWrite(0x42,OV7725_COM3,0xD0); //水平图像左右颠倒
     ack1 = sccb_regWrite(0x42,OV7725_COM5,0x00); //不使用夜间模式
     ack1 = sccb_regWrite(0x42,OV7725_COM7,0x40);  //QVGA
     ack1 = sccb_regWrite(0x42,OV7725_COM8,0xC0);  //关闭自动曝光调整
     ack1 = sccb_regWrite(0x42,OV7725_AECH,0x20);  //手动调整曝光参数
     ack1 = sccb_regWrite(0x42,OV7725_AEC,0x20);
     ack1 = sccb_regWrite(0x42,OV7725_HSTART,0x3F); //0x3f  320  传感器尺寸，以下4个都是QVGA的默认值
     ack1 = sccb_regWrite(0x42,OV7725_HSIZE,0x50);
     ack1 = sccb_regWrite(0x42,OV7725_VSTRT,0x03);  //240
     ack1 = sccb_regWrite(0x42,OV7725_VSIZE,0x78);
     ack1 = sccb_regWrite(0x42,OV7725_HREF,0x00); //好像和HREF信号控制有关，默认值
     ack1 = sccb_regWrite(0x42,OV7725_SCAL0,0x0A); //降采样，采样率为原先的1/4，0XFF就降为1/8
     /* ack1 = sccb_regWrite(0x42,OV7725_AWB_Ctrl0,0xE0); //白平衡，好像是默认值但修改了reserved的内容，注释了 */
     ack1 = sccb_regWrite(0x42,OV7725_DSPAuto,0xff);//摄像头内置图像处理相关参数，全是默认值
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl2,0x0C);//开启水平和垂直方向的白平衡
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl3,0x00);//关闭color bar(好像是用来调试白平衡用的)
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl4,0x00);//自动曝光相关控制，反正已经禁用了，全是默认值
     
     /* ack1 = sccb_regWrite(0x42,OV7725_HOutSize,0x50);  //320 */
     ack1 = sccb_regWrite(0x42,OV7725_HOutSize,0x1E);  //160 -> 120
     /* ack1 = sccb_regWrite(0x42,OV7725_VOutSize,0x78);  //240  */
     ack1 = sccb_regWrite(0x42,OV7725_VOutSize,0x20);  //120 ->64
     
     //以下如果没有注释的寄存器全是默认值
     ack1 = sccb_regWrite(0x42,OV7725_EXHCH,0x00);
     ack1 = sccb_regWrite(0x42,OV7725_GAM1,0x0c); //GAM系列寄存器是为了
     ack1 = sccb_regWrite(0x42,OV7725_GAM2,0x16); //调节gamma曲线
     ack1 = sccb_regWrite(0x42,OV7725_GAM3,0x2a);
     ack1 = sccb_regWrite(0x42,OV7725_GAM4,0x4e);
     ack1 = sccb_regWrite(0x42,OV7725_GAM5,0x61);
     ack1 = sccb_regWrite(0x42,OV7725_GAM6,0x6f);
     ack1 = sccb_regWrite(0x42,OV7725_GAM7,0x7b);
     ack1 = sccb_regWrite(0x42,OV7725_GAM8,0x86);
     ack1 = sccb_regWrite(0x42,OV7725_GAM9,0x8e);
     ack1 = sccb_regWrite(0x42,OV7725_GAM10,0x97);
     ack1 = sccb_regWrite(0x42,OV7725_GAM11,0xa4);
     ack1 = sccb_regWrite(0x42,OV7725_GAM12,0xaf);
     ack1 = sccb_regWrite(0x42,OV7725_GAM13,0xc5);
     ack1 = sccb_regWrite(0x42,OV7725_GAM14,0xd7);
     ack1 = sccb_regWrite(0x42,OV7725_GAM15,0xe8);
     ack1 = sccb_regWrite(0x42,OV7725_SLOP,0x20);
     ack1 = sccb_regWrite(0x42,OV7725_LC_RADI,0x00);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEF,0x13);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_LC_XC,0x08);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEFB,0x14);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEFR,0x17);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_LC_CTR,0x05);//镜头像差矫正相关
     ack1 = sccb_regWrite(0x42,OV7725_BDBase,0x99);//带宽滤波相关
     ack1 = sccb_regWrite(0x42,OV7725_BDMStep,0x03);//带宽滤波相关
     ack1 = sccb_regWrite(0x42,OV7725_SDE,0x04);//饱和度固定
     ack2 = sccb_regWrite(0x42,OV7725_BRIGHT,0x20);  //值越大，图像越白
     ack2 = sccb_regWrite(0x42,OV7725_CNST,0xFF); //环境很暗：0xff  环境很亮：0x00 是硬件二值化的阈值
     ack3 = sccb_regWrite(0x42,OV7725_SIGN,0x06);
     ack3 = sccb_regWrite(0x42,OV7725_UVADJ0,0x11);
     ack3 = sccb_regWrite(0x42,OV7725_UVADJ1,0x02);
     sccb_wait();
     if( (ack1 == 0) && (ack2 == 0) && (ack3 == 0)) 
     {
        gpio_set (PORTA, 17, 0);  //写成功  灯亮 
        sccb_wait();
        ACK = 0;
        break;
     }
     else
     {
        gpio_set (PORTA, 17, 1);  //写失败  灯灭
        sccb_wait();
        ACK = 1;
        continue;
      }
    }
    return ACK;
}
