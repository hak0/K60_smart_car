//////////////////////////////////////////
////         ���ݺ���ΰҵ      ///////////
///   http://shop36538723.taobao.com /////
///                                  ////
///     ��лҰ���ṩ�Ŀ��ļ�          ///
///           2015-4-10               ///
//////////////////////////////////////////
#include "sccb.h"
#include "gpio.h"



/*************************************************************************
*                            
*
*  �������ƣ�sccb_init
*  ����˵������ʼ��SCCB  ����SCL��PTC6 SDA��PTC7
*  ����˵������
*  �������أ���
*
*************************************************************************/
//-------��ʼ��PC6  PC7ΪGPIO--------//
void sccb_init(void)
{
  PORT_PCR_REG(PORTC_BASE_PTR, 6) = (0 | PORT_PCR_MUX(1) );
  PORT_PCR_REG(PORTC_BASE_PTR, 7) = (0 | PORT_PCR_MUX(1) );
}

/************************************************************************
*                            
*
*  �������ƣ�sccb_wait
*  ����˵����SCCB��ʱ����Ӧ̫С
*  ����˵������
*  �������أ���
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
*  �������ƣ�sccb_start
*  ����˵����SCCB����λ
*  ����˵������
*  �������أ���
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
*  �������ƣ�sccb_stop
*  ����˵����SCCBֹͣλ
*  ����˵������
*  �������أ���
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
*  �������ƣ�sccb_sendByte
*  ����˵������SCCB�����Ϸ���һ���ֽ�
*  ����˵����data Ҫ���͵��ֽ�����
*  �������أ���
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
*  �������ƣ�sccb_regWrite
*  ����˵����ͨ��SCCB������ָ���豸��ָ����ַ����ָ������
*  ����˵����device---�豸��  ��д������
*            address---д���ݵļĴ���
*            data---д������
*  �������أ�ack=1δ�յ�Ӧ��(ʧ��)    ack=0�յ�Ӧ��(�ɹ�)
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
    //-----------SCCB�ָ�Ĭ�ϳ�������----------//
    //--PCLK:73ns   HREF:63.6us   VSYN:16.64ms--//
    //--Ĭ�ϸ���ɨ�裬ȫ�ֱ���Ϊ640*480���ɼ�VSYN�ֱ�����640*240  
    //--��ÿ����HREF֮����640��PCLK
    //--��ÿ����VSYN֮����240��HREF    
    //sccb_regWrite(0x42,0x11,0x00);   
    //sccb_regWrite(0x42,0x14,0x04);
    //sccb_regWrite(0x42,0x28,0x20);
    //---------------------------------------//
    u8 ack1,ack2,ack3;
    u8 ACK = 1;
    for(u8 sccb_time=0; sccb_time<10; sccb_time++)
    {
      ack1 = sccb_regWrite(0x42,0x12,0x80);    //COM7 , ��λ���мĴ���
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
//      ack3 = sccb_regWrite(0x42,0x15,0x20);    //COM10  ����PCLK�ȷ�ת
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
     ack1 = sccb_regWrite(0x42,OV7725_COM4,0x41);  //41 4����Ƶ
     /* ack1 = sccb_regWrite(0x42,OV7725_COM4,0xC1);  //81 8����Ƶ,������أ����� */
     ack1 = sccb_regWrite(0x42,OV7725_CLKRC,0x00); //����ͷʱ��������ٶ�����
     //ack1 = sccb_regWrite(0x42,OV7725_COM2,0x03);
     ack1 = sccb_regWrite(0x42,OV7725_COM3,0xD0); //ˮƽͼ�����ҵߵ�
     ack1 = sccb_regWrite(0x42,OV7725_COM5,0x00); //��ʹ��ҹ��ģʽ
     ack1 = sccb_regWrite(0x42,OV7725_COM7,0x40);  //QVGA
     ack1 = sccb_regWrite(0x42,OV7725_COM8,0xC0);  //�ر��Զ��ع����
     ack1 = sccb_regWrite(0x42,OV7725_AECH,0x20);  //�ֶ������ع����
     ack1 = sccb_regWrite(0x42,OV7725_AEC,0x20);
     ack1 = sccb_regWrite(0x42,OV7725_HSTART,0x3F); //0x3f  320  �������ߴ磬����4������QVGA��Ĭ��ֵ
     ack1 = sccb_regWrite(0x42,OV7725_HSIZE,0x50);
     ack1 = sccb_regWrite(0x42,OV7725_VSTRT,0x03);  //240
     ack1 = sccb_regWrite(0x42,OV7725_VSIZE,0x78);
     ack1 = sccb_regWrite(0x42,OV7725_HREF,0x00); //�����HREF�źſ����йأ�Ĭ��ֵ
     ack1 = sccb_regWrite(0x42,OV7725_SCAL0,0x0A); //��������������Ϊԭ�ȵ�1/4��0XFF�ͽ�Ϊ1/8
     /* ack1 = sccb_regWrite(0x42,OV7725_AWB_Ctrl0,0xE0); //��ƽ�⣬������Ĭ��ֵ���޸���reserved�����ݣ�ע���� */
     ack1 = sccb_regWrite(0x42,OV7725_DSPAuto,0xff);//����ͷ����ͼ������ز�����ȫ��Ĭ��ֵ
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl2,0x0C);//����ˮƽ�ʹ�ֱ����İ�ƽ��
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl3,0x00);//�ر�color bar(�������������԰�ƽ���õ�)
     ack1 = sccb_regWrite(0x42,OV7725_DSP_Ctrl4,0x00);//�Զ��ع���ؿ��ƣ������Ѿ������ˣ�ȫ��Ĭ��ֵ
     
     /* ack1 = sccb_regWrite(0x42,OV7725_HOutSize,0x50);  //320 */
     ack1 = sccb_regWrite(0x42,OV7725_HOutSize,0x1E);  //160 -> 120
     /* ack1 = sccb_regWrite(0x42,OV7725_VOutSize,0x78);  //240  */
     ack1 = sccb_regWrite(0x42,OV7725_VOutSize,0x20);  //120 ->64
     
     //�������û��ע�͵ļĴ���ȫ��Ĭ��ֵ
     ack1 = sccb_regWrite(0x42,OV7725_EXHCH,0x00);
     ack1 = sccb_regWrite(0x42,OV7725_GAM1,0x0c); //GAMϵ�мĴ�����Ϊ��
     ack1 = sccb_regWrite(0x42,OV7725_GAM2,0x16); //����gamma����
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
     ack1 = sccb_regWrite(0x42,OV7725_LC_RADI,0x00);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEF,0x13);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_LC_XC,0x08);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEFB,0x14);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_LC_COEFR,0x17);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_LC_CTR,0x05);//��ͷ���������
     ack1 = sccb_regWrite(0x42,OV7725_BDBase,0x99);//�����˲����
     ack1 = sccb_regWrite(0x42,OV7725_BDMStep,0x03);//�����˲����
     ack1 = sccb_regWrite(0x42,OV7725_SDE,0x04);//���Ͷȹ̶�
     ack2 = sccb_regWrite(0x42,OV7725_BRIGHT,0x20);  //ֵԽ��ͼ��Խ��
     ack2 = sccb_regWrite(0x42,OV7725_CNST,0xFF); //�����ܰ���0xff  ����������0x00 ��Ӳ����ֵ������ֵ
     ack3 = sccb_regWrite(0x42,OV7725_SIGN,0x06);
     ack3 = sccb_regWrite(0x42,OV7725_UVADJ0,0x11);
     ack3 = sccb_regWrite(0x42,OV7725_UVADJ1,0x02);
     sccb_wait();
     if( (ack1 == 0) && (ack2 == 0) && (ack3 == 0)) 
     {
        gpio_set (PORTA, 17, 0);  //д�ɹ�  ���� 
        sccb_wait();
        ACK = 0;
        break;
     }
     else
     {
        gpio_set (PORTA, 17, 1);  //дʧ��  ����
        sccb_wait();
        ACK = 1;
        continue;
      }
    }
    return ACK;
}
