//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///           2015-4-10               ///
//////////////////////////////////////////

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
/*
 * Include 用户自定义的头文件
 */
#include  "gpio.h"      //IO口操作
#include  "delay.h"      //IO口操作
#include  "dma.h"
#include  "uart.h"      //串口
#include  "adc.h"       //ADC模块
#include  "FTM.h"       //FTM模块（FTM0：电机控制 / 通用 /PWM     FTM1、2：正交解码 / 通用 /PWM ）
#include  "PIT.h"       //周期中断计时器
#include  "lptmr.h"     //低功耗定时器(延时)
#include  "exti.h"      //EXTI外部GPIO中断
#include  "arm_math.h"  //DSP库
#include  "sccb.h"      //SCCB库
#include  "common.h"

#include "LQ12864.h"  //自己加的


#define ROW 60              //行数
#define COL 15              //列数  为消除消隐区
//#define PicSize  ROW*COL      //图像大小
#define _UART_BlueTooth_      //若用蓝牙进行通信 去掉注释该语句
#ifdef _UART_BlueTooth_
  #define  UART    UART0
#else
  #define  UART    UART4
#endif
/***************** ucos 专用 *****************/
#define USOC_EN     0u      //0为禁止uC/OS，大于0则启动uC/OS
#if USOC_EN > 0u
#include  "ucos_ii.h"  		//uC/OS-II系统函数头文件
#include  "BSP.h"			//与开发板相关的函数
#include  "app.h"			//用户任务函数
#endif  //if  USOC_EN > 0
#endif  //__INCLUDE_H__
