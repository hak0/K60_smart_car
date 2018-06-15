//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///           2015-4-10               ///
//////////////////////////////////////////

#include "common.h"
#include "include.h"

char data_receive[25];
char tof_receive[5];
u8 tof_num_flag = 0;
int index_bt = 0;
int index_tof = 0;
unsigned char Is_SendPhoto = 0;
unsigned char V_Cnt = 0;
extern u16 tof_value;            //tof测得的距离
extern unsigned short DuoCenter; ////舵机中间值 120HZ
extern unsigned short dianjispeed;
extern u8 ensend; //允许发送

extern u16 servPram;
extern u16 dPram;
extern u8 enpwm;
extern u8 speedmodi;

extern void run();

extern u32 rowCnt; //行计数
extern u8 Buffer1[ROW][COL];
u8 SampleFlag = 0;
extern u8 VSYN_Flag;

unsigned char flag_1ms = 0;
u8 TIME1flag_100ms = 0;
/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：USART4_IRQHandler
*  功能说明：串口5 中断 接收 服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void USART4_IRQHandler(void)
{
    uint8 ch;
    //DisableInterrupts;		    //关总中断
    //接收一个字节数据并回发
    ch = uart_getchar(UART4); //接收到一个数据
    data_receive[index_bt] = ch;
    index_bt++;
    if (data_receive[0] == '#') //标识头
    {
        if (ch == '$') {
            if (data_receive[1] == 'D') //舵机中值
            {
                DuoCenter = (data_receive[2] - 48) * 1000 + (data_receive[3] - 48) * 100 + (data_receive[4] - 48) * 10 + (data_receive[5] - 48);
                FTM_PWM_Duty(FTM2, CH1, DuoCenter);
            }
            if (data_receive[1] == 'Y') // 允许摄像头上传
            {
                ensend = 1;
            }
            if (data_receive[1] == 'N') // 禁止摄像头上传
            {
                ensend = 0;
            }
            if (data_receive[1] == 'X') // 允许PWM值上传
            {
                enpwm = 1;
            }
            if (data_receive[1] == 'Z') // 禁止PWM值上传
            {
                enpwm = 0;
            }
            if (data_receive[1] == 'P') //舵机比例系数
            {
                servPram = (data_receive[2] - 48) * 100 + (data_receive[3] - 48) * 10 + (data_receive[4] - 48);
            }
            if (data_receive[1] == 'F') //舵机微分系数
            {
                dPram = (data_receive[2] - 48) * 100 + (data_receive[3] - 48) * 10 + (data_receive[4] - 48);
            }
            for (index_bt = 0; index_bt < 10; index_bt++)
                data_receive[index_bt] = 0x00;
            index_bt = 0;
        }
        if (index_bt > 12)
            index_bt = 0;
    } else {
        for (index_bt = 0; index_bt < 10; index_bt++)
            data_receive[index_bt] = 0x00;
        index_bt = 0;
    }
    //EnableInterrupts;		    //开总中断
    //uart_putchar(UART0,'A');
}

void USART3_IRQHandler(void)
{
    //TODO:利用电脑修改TOF设置，以最高频率发送
    uint8 ch;
    ch = uart_getchar(UART3);
    if (ch == '\n') { // 起始标记，开始保存接收的数据
        for (index_tof = 0; index_tof < 5; index_tof++) {
            tof_receive[index_tof] = 0x00; //先清空记录接收数据的数组
        }
        index_tof = 0;      //下标归位
        tof_num_flag = 1;   //表示接下来的数据是数字，保存
    } else if ((ch == 'm') && (tof_num_flag == 1)) { //结束标记
        tof_num_flag = 0;   //表示接下来的数据是字母和换行符，丢弃
        tof_value = 0;
        for (; index_tof >= 0; index_tof--) { //将数组中的值转换为数字并保存到对应变量
            tof_value += tof_receive[index_tof];
            tof_value *= 10;
            index_tof--;
        }
    } else if (tof_num_flag) { //保存接收的数据到数组
        tof_receive[index_tof] = ch - '0';
        index_tof++;
    }
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：PIT0_IRQHandler
*  功能说明：PIT0 定时中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
#if 1
void PIT0_IRQHandler(void) //1ms
{
    // LED_turn(LED1);             //LED1反转
    PIT_Flag_Clear(PIT0); //清中断标志位
    flag_1ms = 1;         //1ms中断标志
    run();                //测速函数
}
#else
void PIT0_IRQHandler(void)
{
    PIT_Flag_Clear(PIT0); //清中断标志位
}
#endif

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：SysTick_Handler
*  功能说明：系统滴答定时器中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：HardFault_Handler
*  功能说明：硬件上访中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void HardFault_Handler(void)
{
    while (1) {
        printf("\n****硬件上访错误!!!*****\r\n\n");
    }
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：PendSV_Handler
*  功能说明：PendSV（可悬起系统调用）中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void PendSV_Handler(void)
{
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：PORTE_IRQHandler
*  功能说明：PORTE端口中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要自己初始化来清除
*
*************************************************************************/

void PORTE_IRQHandler()
{
    unsigned int lie;
    //---VSYN场中断处理
    if (PORTE_ISFR & (1 << 1)) //PTE1触发中断  场中断
    {
        PORTE_ISFR |= (1 << 1); //写1清中断标志位
        rowCnt = 0;
        SampleFlag = 1;
    }
    //---HREF行中断处理
    if (PORTE_ISFR & (1 << 0)) //PTE0触发中断
    {
        PORTE_ISFR |= (1 << 0); //写1清中断标志位
        if (SampleFlag == 0)    //不足一场时返回
        {
            return;
        }
        //-------------DMA初始化通道4，数据源为PTD，每次存在数组ImageBuf[]指针中，PCLK接PTA19触发，每次传输1个字节，每次触发传输300次，上升沿触发
        else {
            if (rowCnt > 10) {
                for (lie = 0; lie < 70; lie++) //130  延时修改，可以调节图像的左右
                {
                    asm("nop");
                    asm("nop");
                }
                DMA_PORTx2BUFF_Init(DMA_CH4, (void*)&PTD_BYTE0_IN, Buffer1[rowCnt - 11], PTA19, DMA_BYTE1, COL, DMA_rising_down);
                DMA_EN(DMA_CH4);
            }
            rowCnt++;
            if (rowCnt > ROW + 11) {
                DisableInterrupts;
                rowCnt = 0;
                DMA_DIS(DMA_CH4);
                VSYN_Flag = 1;
                SampleFlag = 0;
            }
        }
    }
}

/*************************************************************************
*                            岱默科技DEMOK Kinetis开发小组
*
*  函数名称：DMA_CH4_Handler
*  功能说明：DMA通道4的中断服务函数
*  参数说明：无
*  函数返回：无
*
*************************************************************************/
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4); //清除通道传输中断标志位    (这样才能再次进入中断)
    DMA_IRQ_DIS(DMA_CH4);   //禁止DMA   等待下一次行中断来临开启DMA
}

/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM0_IRQHandler
*  功能说明：FTM0输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/
void FTM0_IRQHandler()
{
}
/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM2_IRQHandler
*  功能说明：FTM2输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/
void FTM2_IRQHandler()
{
}
/*************************************************************************
*                             岱默科技DEMOK Kinetis开发小组
*
*  函数名称：FTM1_IRQHandler
*  功能说明：FTM1输入捕捉中断服务函数
*  参数说明：无
*  函数返回：无
*  备    注：引脚号需要根据自己初始化来修改，参考现有的代码添加自己的功能
*
*************************************************************************/
void FTM1_IRQHandler()
{
    u8 s = FTM1_STATUS; //读取捕捉和比较状态  All CHnF bits can be checked using only one read of STATUS.
    u8 n;
    FTM1_STATUS = 0x00; //清中断标志位

    n = 0;
    if (s & (1 << n)) {
        FTM_CnSC_REG(FTM1_BASE_PTR, n) &= ~FTM_CnSC_CHIE_MASK; //禁止输入捕捉中断
        /*     用户任务       */
        //LED_turn(LED1);                             //翻转LED1
        printf("\nFTM1发送中断\n");

        /*********************/
        //不建议在这里开启输入捕捉中断，而是在main函数里根据需要来开启
        //通道 CH0、CH1、Ch2、Ch3 有滤波器
        //FTM_CnSC_REG(FTM1_BASE_PTR,n) |= FTM_CnSC_CHIE_MASK;  //开启输入捕捉中断
        //delayms(10);        //因为输入的信号跳变过程不稳定，容易触发多次输入捕捉，所以添加延时
        //但考虑到中断不应该过长延时，所以开输入捕捉中断就放在main函数里，根据需要来开启
    }

    n = 1;
    if (s & (1 << n)) {
        FTM_CnSC_REG(FTM1_BASE_PTR, n) &= ~FTM_CnSC_CHIE_MASK; //禁止输入捕捉中断
        /*     用户任务       */

        /*********************/
        //不建议在这里开启输入捕捉中断
        //FTM_CnSC_REG(FTM1_BASE_PTR,n) |= FTM_CnSC_CHIE_MASK;  //开启输入捕捉中断
    }
}

