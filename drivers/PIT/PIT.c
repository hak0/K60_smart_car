/******************** (C) COPYRIGHT 2012-2013 岱默科技 DEMOK*******************
 * 文件名         ：PIT.c
 * 描述           ：PIT周期中断计时器，定时中断

 * 实验平台       ： 岱默科技DEMOK Kinetis开发板
 * 作者           ： 岱默科技DEMOK Kinetis开发小组

 * 淘宝店铺       ： http://shop60443799.taobao.com/
 * 技术交流邮箱   ： 1030923155@qq.com 
 * 技术交流QQ群   ： 103360642

 * 最后修订时间    ：2012-11-5
 * 修订内容        ：无
**********************************************************************************/


#include "common.h"
#include  "PIT.h"     //周期中断计时器


/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：pit_init
*  功能说明：PITn定时中断
*  参数说明：PITn        模块号（PIT0~PIT3）
             cnt         延时时间(单位为bus时钟周期)
*  函数返回：无
*
*************************************************************************/
void pit_init(PITn pitn, u32 cnt)
{
    //PIT 用的是 Bus Clock 总线频率
    //溢出计数 = 总线频率 * 时间

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //使能PIT时钟

    /* PIT模块控制 PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //使能PIT定时器时钟 ，调试模式下继续运行

    /* 定时器加载值设置 Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt;                                          //设置溢出中断时间

    //定时时间到了后，TIF 置 1 。写1的时候就会清0
    PIT_Flag_Clear(pitn);                                             //清中断标志位

    /* 定时器控制寄存器 Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //使能 PITn定时器,并开PITn中断

    enable_irq(pitn + 68);			                                //开接收引脚的IRQ中断
}


/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：pit_init
*  功能说明：PITn定时中断
*  参数说明：PITn        模块号（PIT0~PIT3）
             cnt         延时时间(单位为bus时钟周期)
*  函数返回：无
*
*************************************************************************/
void pit_dma_init(PITn pitn, u32 cnt)
{
    //PIT 用的是 Bus Clock 总线频率
    //溢出计数 = 总线频率 * 时间

    /* 开启时钟*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //使能PIT时钟

    /* PIT模块控制 PIT Module Control Register (PIT_MCR) */
    //PIT_MCR         &=~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );       //使能PIT定时器时钟 ，调试模式下继续运行
    PIT_MCR = 0;

    /* 定时器加载值设置 Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt;                                          //设置溢出中断时间

    //定时时间到了后，TIF 置 1 。写1的时候就会清0
    PIT_Flag_Clear(pitn);                                             //清中断标志位

    /* 定时器控制寄存器 Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= (0
                        | PIT_TCTRL_TEN_MASK  //使能 PITn定时器
                        | PIT_TCTRL_TIE_MASK  //开PITn中断
                       );

    enable_irq(pitn + 68);			                                //开接收引脚的IRQ中断
}