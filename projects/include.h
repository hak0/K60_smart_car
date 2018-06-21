//////////////////////////////////////////
////         ���ݺ���ΰҵ      ///////////
///   http://shop36538723.taobao.com /////
///           2015-4-10               ///
//////////////////////////////////////////

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "gpio.h"      //IO�ڲ���
#include  "delay.h"      //IO�ڲ���
#include  "dma.h"
#include  "uart.h"      //����
#include  "adc.h"       //ADCģ��
#include  "FTM.h"       //FTMģ�飨FTM0��������� / ͨ�� /PWM     FTM1��2���������� / ͨ�� /PWM ��
#include  "PIT.h"       //�����жϼ�ʱ��
#include  "lptmr.h"     //�͹��Ķ�ʱ��(��ʱ)
#include  "exti.h"      //EXTI�ⲿGPIO�ж�
#include  "arm_math.h"  //DSP��
#include  "sccb.h"      //SCCB��
#include  "common.h"

#include "LQ12864.h"  //�Լ��ӵ�


#define ROW 60              //����
#define COL 15              //����  Ϊ����������
//#define PicSize  ROW*COL      //ͼ���С
#define _UART_BlueTooth_      //������������ͨ�� ȥ��ע�͸����
#ifdef _UART_BlueTooth_
  #define  UART    UART0
#else
  #define  UART    UART4
#endif
/***************** ucos ר�� *****************/
#define USOC_EN     0u      //0Ϊ��ֹuC/OS������0������uC/OS
#if USOC_EN > 0u
#include  "ucos_ii.h"  		//uC/OS-IIϵͳ����ͷ�ļ�
#include  "BSP.h"			//�뿪������صĺ���
#include  "app.h"			//�û�������
#endif  //if  USOC_EN > 0
#endif  //__INCLUDE_H__
