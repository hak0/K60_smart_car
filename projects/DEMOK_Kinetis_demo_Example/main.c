//TODO:正式运行时将options-compiler中的optimization开高
//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///           2016-4-10               ///
//////////////////////////////////////////
// 940 1120 1300
#include "common.h"
#include "include.h"

#define tof_value_min 0 //最小距离值，再小就触发灭灯

#define servMotorNear 1160 //730 //770  //1580
//灭灯舵机最近极限
#define servMotorFar 550 //1030 //1070 //1050 //1840
//灭灯舵机最远极限
u16 servValue = servMotorNear; //舵机位置

#define cmosrow 60
#define cmoscol 120    //该值必须是8的整数倍 35*8 //实际处理数据分辨率 280*150
#define comscolbyte 15 // =cmoscol/8
#define duty_max 10000

//定义方向
#define left 0
#define right 1
#define front 0
#define back 2
//这样，描述方向可以用front|left这种格式

u8 Buffer1[ROW][COL] = { 0 }; //37*8=296
/* 通过DMA采集(在isr.c中行中断中触发DMA采集)二维图像存储于Buffer1数组中，
 * ROW,COL在Include.h中定义 */

u8 cmos[cmosrow][cmoscol] = { 0 };       //进行处理的数据
u8 cmos_copy[cmosrow][cmoscol] = { 0 };  //进行处理的数据
u8 cmos_update_flag = 0;                 //用于调试，置1后会刷新cmos的状况
u8 cmos_cnst = 0x86, cmos_bright = 0x6A; //cmos的对比度和亮度
u8 VSYN_Flag = 0;                        //完成一场标识位  0为未完成；1为完成

u8 ensend; //允许发送
u8 enpwm;
u8 entof;
u8 enccd;

// 光源位置
u16 light_x;
u16 light_y;
u8 cmos_see_light;

//
s32 TimeCount = 0;           //1ms中断标志
s32 TimeCount_Max = 1000000; //计时最大时间为1e6ms=1000s

s16 g_nMotorPulse[2];
s32 g_nMotorPulseSigma[2];
s16 g_MotorPWM[2] = { 0, 0 }; // 左右电机PWM值
u8 g_MotorBrake[2];           // 左右电机刹车信号
s16 speed[2] = { 0, 0 };      // 左右电机设置速度值

char uart_dma_recv[16];
u16 tof_value = 0; //tof测得的距离值

volatile u32 rowCnt = 0; //行计数

u8 encoder_dir;        //编码器方向
u8 encoder_select = 1; //选择编码器,0表示左1表示右

u32 brake_start_time = 0;
u32 back_start_time = 0;
u8 move_back_flag = 0;
u8 move_back_direction = 0;

u16 CCD[128]; //CCD AD亮度值
u16 ccd_max_light;
u32 ccd_light_average;
u32 ccd_light_width;
u32 ccd_light_centre;
u16 ccd_light_threshorld = 150;
u16 ccd_exposuretime = 160;
u8 ccd_upload_flag = 0;
u8 ccd_last_direction = 0;

u16 distance[3] = { 0 };
u16 voltage[3] = { 0 };
u8 near_flag = 0;
u8 near_distance_range = 40;

u16 pwm_max = 8000;
u16 pwm_near = 3000;
u16 pwm_min = 1200;//1200//1500
u16 pwm_back = 1200;
u16 pwm_rotate[2] = { 1500, -1300 };//1500 0//

u8 too_much_brake_flag = 0;

//-----------------函数声明------------------
void image_erode(unsigned char (*t)[cmoscol], unsigned char (*temp)[cmoscol], unsigned char b);
void ImageProc();
void GPIO_Init();
void PWM_Init();
void UART_Init();
void EXTI_Init();
void Image_Init();
void run();
void update_pwm();
void CountThreshole(void);
void datatrans();
void uart_dma_init();
void PID();
void DMAProc();
void decide_speed();
void get_distance();
void myDMA_Start(uint8 DMA_CHn);
void myDMA_Close(uint8 DMA_CHn);
void ccd_init();
void ccd_gather();
void ccd_upload();
void ccd_proc();
//-----------------主函数--------------------
void main()
{
    u8 sccb_ack;

    //debug标志
    ensend = 0;
    entof = 0;
    enccd = 0;

    //开始初始化
    DisableInterrupts; //关闭中断开始初始化数据
    GPIO_Init();       //------GPIO初始化  用于指示SCCB状态

    PWM_Init();            //-----PWM初始化，用于初始化舵机和电机的输出
    UART_Init();           //-----UART初始化，用于初始化串口
    
    pit_init_ms(PIT0, 8); //初始化PIT0，定时时间为： 10ms
    ADC_Init(ADC0);        //初始化AD接口，ADC0，用于线阵CCD和红外测距
    EnableInterrupts;
    // 初始化结束
    gpio_set(PORTC, 18, 1); //初始化，LED点亮

    uart_irq_EN(UART4);
    uart_dma_init(); //----TOF 串口DMA初始化
    uart_putchar(UART4, 'S');
    uart_putchar(UART4, 't');
    uart_putchar(UART4, 'a');
    uart_putchar(UART4, 'r');
    uart_putchar(UART4, 't');
    uart_putchar(UART4, '.');
    uart_putchar(UART4, '.');

    rowCnt = 0;
    while (1) {
        get_distance();
        if (ccd_upload_flag) {
            ccd_upload();
            ccd_upload_flag = 0;
        }
        DisableInterrupts;
        decide_speed(); //根据光源位置状态决定PWM值
        update_pwm(); //更新输出到驱动板的pwm值
        EnableInterrupts;
    }
}

void GPIO_Init()
{
    gpio_init(PORTB, 16, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受对管(寻灯时后轮防止流星锤)(左后)
    gpio_init(PORTB, 17, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受对管(寻灯时后轮防止流行锤)(右后)
    /* gpio_init(PORTE, 12, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻 */
    //对管
    gpio_init(PORTE, 2, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻
    gpio_init(PORTE, 3, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻
    gpio_init(PORTA, 12, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受对管(倒车)
    gpio_init(PORTA, 13, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受对管(倒车)
    /* gpio_init(PORTB, 17, GPO, encoder_select);                                //初始化PTB17为电平输出，，选择当前检测的编码器 */
    gpio_init(PORTA, 17, GPO, 1);                                             //初始化PTE0为高电平输出---LED0
    gpio_set(PORTA, 17, 1);                                                   //设置PTE0为高电平输出，LED0灭
    gpio_init(PORTE, 7, GPO, 0);                                              //初始化PTB17为电平输出，，选择当前检测的编码器
    PORT_PCR_REG(PORTE_BASE_PTR, 7) |= PORT_PCR_PE_MASK & !PORT_PCR_PS_MASK;  //开弱下拉
    gpio_init(PORTE, 8, GPO, 0);                                              //初始化PTB17为电平输出，，选择当前检测的编码器
    PORT_PCR_REG(PORTE_BASE_PTR, 8) |= PORT_PCR_PE_MASK & !PORT_PCR_PS_MASK;  //开弱下拉
    gpio_init(PORTE, 9, GPO, 0);                                              //初始化PTB17为电平输出，，选择当前检测的编码器
    PORT_PCR_REG(PORTE_BASE_PTR, 9) |= PORT_PCR_PE_MASK & !PORT_PCR_PS_MASK;  //开弱下拉
    gpio_init(PORTE, 10, GPO, 0);                                             //初始化PTB17为电平输出，，选择当前检测的编码器
    gpio_init(PORTE, 11, GPO, 0);                                             //初始化PTB17为电平输出，，选择当前检测的编码器
    gpio_init(PORTE, 12, GPO, 0);                                             //PTE11用于输出计时信号
    PORT_PCR_REG(PORTE_BASE_PTR, 10) |= PORT_PCR_PE_MASK & !PORT_PCR_PS_MASK; //开弱下拉
                                                                              //如果之后初始化成功，会点亮灯。
}

void PWM_Init()
{
    FTM_PWM_init(FTM2, CH1, 50, servValue); //初始化FTM0_CH0输出频率为50HZ,占空比为servValue ：舵机 FTM2_CH1对应PTB19口
    // 要改变占空比，用FTM_PWM_Duty
    FTM_PWM_init(FTM0, CH0, 10000, 0); //初始化FTM0_CH0输出频率为10KHZ,占空比dianjispeed ：FTM0_CH0对应PTC1口,左侧电机
    FTM_PWM_init(FTM0, CH1, 10000, 0); //初始化FTM0_CH0输出频率为10KHZ,占空比dianjispeed ：FTM0_CH0对应PTC2口,右侧电机
}

void UART_Init()
{
    uart_init(UART4, 115200); //115200);//115200);      //初始化串口4-蓝牙串口 波特率为115200
    uart_init(UART3, 9600);   //115200);//115200);      //初始化串口3-TOF串口 波特率为9600
}

void EXTI_Init()
{
    // 全是和面阵cmos相关的函数
    //TODO:检查是上升沿还是下降沿
    //----初始化外部中断---//
    /* exti_init(PORTE, 0, rising_down); //HREF----PORTE0 端口外部中断初始化 ，上升沿触发中断，内部下拉 */
    exti_init(PORTE, 0, falling_down); //HREF----PORTE0 端口外部中断初始化 ，上升沿触发中断，内部上拉，但上升沿会来不及初始化DMA然后出问题，还是用下降沿吧，这样会丢失一行数据
    exti_init(PORTE, 1, falling_down); //VSYN----PORTE1 端口外部中断初始化 ，下降沿触发中断，内部下拉
}
void Image_Init()
{
    u32 i, j;
    for (i = 0; i < ROW; i++) {
        for (j = 0; j < COL; j++) {
            Buffer1[i][j] = 0;
        }
    }
}

void datatrans() //buffer1 的 50~100行
{
    uint16 m, n;
    uint8 colour[2] = { 0, 240 }; //0 和 1 分别对应的颜色
    int offset = 9;
    for (m = 0; m < cmosrow; m++) //150行
    {
        for (n = 0; n < comscolbyte; n++) // 35*8 = 280列
        {
            cmos[m][(8 * n - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 7) & 0x01];
            cmos[m][(8 * n + 1 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 6) & 0x01];
            cmos[m][(8 * n + 2 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 5) & 0x01];
            cmos[m][(8 * n + 3 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 4) & 0x01];
            cmos[m][(8 * n + 4 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 3) & 0x01];
            cmos[m][(8 * n + 5 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 2) & 0x01];
            cmos[m][(8 * n + 6 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 1) & 0x01];
            cmos[m][(8 * n + 7 - offset) % cmoscol] = colour[(Buffer1[m][n + 1] >> 0) & 0x01];
        }
    }
}

void get_light_position()
{
    u32 xsum = 0, ysum = 0;
    u16 i = 0, j = 0, cnt = 0;
    /* 确定光源位置 */

    for (i = 0; i < cmosrow; i++) {
        for (j = 0; j < cmoscol; j++) {
            if (cmos[i][j] != 0) {
                cnt++;
                ysum += i;
                xsum += j;
            }
        }
    }
    light_x = xsum / cnt;
    light_y = ysum / cnt;
    cmos_see_light = (cnt > 10) ? 1 : 0;
}

void ImageProc()
{ //unsigned char  ll=1,rr=1;
    u8 bit_cnt = 0;
    u8 send_buffer = 0;
    u32 i, j;
    s32 servPWMDuty;   //舵机PWM占空比
    DisableInterrupts; //禁止中断
    datatrans();       //把Buffer1的数据移到cmos
    //// 计算光源位置
    get_light_position();
    //// 发送原始数据到上位机
    if (ensend) {
        uart_putchar(UART4, 0xFF); //图像头
        for (i = 0; i < cmosrow; i++) {
            for (j = 0; j < cmoscol; j++) {
                send_buffer = send_buffer << 1;
                send_buffer += (cmos[i][j] != 0);
                if (++bit_cnt >= 4) {
                    bit_cnt = 0;
                    uart_putchar(UART4, send_buffer);
                    send_buffer = 0;
                }
            }
        }
        uart_putchar(UART4, cmos_see_light);
        uart_putchar(UART4, light_x >> 8);
        uart_putchar(UART4, light_x & 0xFF);
        uart_putchar(UART4, light_y & 0xFF);
    }
    memcpy(cmos_copy, cmos, sizeof(cmos));
    image_erode(cmos_copy, cmos, 0); //腐蚀1像素
}

void get_distance()
{
    int i;
    //AD转换
    voltage[0] = ADC_Ave(ADC0, ADC0_SE8, ADC_12bit, 20);  //读取AD0数据
    voltage[1] = ADC_Ave(ADC0, ADC0_SE9, ADC_12bit, 20);  //读取AD1数据
    voltage[2] = ADC_Ave(ADC0, ADC0_SE12, ADC_12bit, 20); //读取AD2数据
    for (i = 0; i < sizeof(voltage); i++) {
        if (voltage[i] < 400)
            voltage[i] = 400;
        if (voltage[i] > 4000)
            voltage[i] = 4000;
    }
    distance[0] = 1 / ((voltage[0] - 399) / 4000.0 * 0.1);
    distance[1] = 1 / ((voltage[1] - 399) / 3000.0 * 0.067);
    distance[2] = 1 / ((voltage[2] - 399) / 4000.0 * 0.1);
}
void run()
{
#if 0 //测速相关
    if (TimeCount % 20 == 0) //20ms读一次速度值
    {
        encoder_dir = gpio_get(PORTB, 16) ^ encoder_select; //编码器左右两个方向相反，和选通位异或后就变得相同
        //encoder_dir = !encoder_dir; //如果前进后退的方向相反，就认为取反

        //encoder_select
        g_nMotorPulse[encoder_select] = encoder_dir ? FTM1_CNT : -FTM1_CNT;  //更新对应的测速变量
        g_nMotorPulseSigma[encoder_select] += g_nMotorPulse[encoder_select]; //更新对应测路程变量
        encoder_select = !encoder_select;                                    //编码器选通位取反
        gpio_set(PORTB, 17, encoder_select);                                 //编码器选通位电平输出,切换到另一侧编码器
        FTM1_CNT = 0;                                                        //编码器计数变量重置
    }
    if (TimeCount % 40 == 0) {
        //每读取一次左右编码器的速度值就利用PID更新一次PWM
        /* PID(); */
    }
#endif
    /* if (TimeCount % 10 == 0) */
    if (TimeCount >= 1000)
        TimeCount = 0; //每1s清零计数变量
}

// 还需要单独加上刹车的函数，如果目标速度和当前速度相差太大，可直接调用刹车
void update_pwm()
{
    /* 根据g_xxxMotorPWM调整施加到电机驱动的pwm值 */
    /* 左侧PTE7向前,PTE9向后,数字量设置 */
    const u8 DIRpins[4] = { 7, 8, 9, 10 }; //左前，右前，左后，右后
    const CHn PWMCH[2] = { CH0, CH1 };

    for (u8 side = left; side <= right; side++) {
        if (g_MotorBrake[side]) { //刹车
            gpio_set(PORTE, DIRpins[side | front], 0);
            gpio_set(PORTE, DIRpins[side | back], 0);
        } else if (g_MotorPWM[side] > 0) { //向前
            gpio_set(PORTE, DIRpins[side | front], 1);
            gpio_set(PORTE, DIRpins[side | back], 0);
        } else if (g_MotorPWM[side] < 0) { // 向后
            gpio_set(PORTE, DIRpins[side | front], 0);
            gpio_set(PORTE, DIRpins[side | back], 1);
        } else { // 停车
            gpio_set(PORTE, DIRpins[side | front], 1);
            gpio_set(PORTE, DIRpins[side | back], 1);
        }
        /* PWM信号设置 */
        if (g_MotorBrake[side])
            FTM_PWM_Duty(FTM0, PWMCH[side], 0);
        else
            FTM_PWM_Duty(FTM0, PWMCH[side], abs(g_MotorPWM[side]));
    }
}

void PID()
{
#if 0
    /*--------------------------------------------------------------*/
    /* 实际上是PI,没用到微分 */
    static double e0[2] = { 0, 0 }, e1[2] = { 0, 0 }, y0[2] = { 0, 0 }, y1[2] = { 0, 0 };
    const u16 duty = 1;
    const double Ki = 1;    //选择比例积分调节器
    const double Kp = 0.25; //比例系数
    const double a0 = Kp * (1.0 + Ki);
    const double a1 = Kp;
    for (u8 side = left; side <= right; side++) {
        y1[side] = y0[side];
        y0[side] = g_nMotorPulse[side];
        e0[side] = speed[side] - y0[side];
        e1[side] = speed[side] - y1[side];
        g_MotorPWM[side] += (a0 * e0[side] - a1 * e1[side]) * duty; //PI控制器的算法
                                                                   // 可能需要设置最小值和最大值
        if (g_MotorPWM[side] >= 8000)
            g_MotorPWM[side] = 8000;
        if (g_MotorPWM[side] <= -8000)
            g_MotorPWM[side] = -8000;
    }
#else
    for (u8 side = left; side <= right; side++) {
        if (g_nMotorPulse[side] < speed[side])
            g_MotorPWM[side] += 100;
        if (g_nMotorPulse[side] > speed[side])
            g_MotorPWM[side] -= 100;
        // 可能需要设置最小值和最大值
        if (g_MotorPWM[side] >= 8000)
            g_MotorPWM[side] = 8000;
        if (g_MotorPWM[side] <= -8000)
            g_MotorPWM[side] = -8000;
    }
#endif
}

void uart_dma_init()
{
    UART3_C2 |= UART_C2_RIE_MASK;   /* 使能UART发送中断或者DMA请求 */
    UART3_C5 |= UART_C5_RDMAS_MASK; /* 打开UART发送的DMA请求 */
    myDMA_Config(1, DMA_UART3_Rx, (u32)&UART3_D, (u32)uart_dma_recv, 4);
    myDMA_Start(1);
    DMA_IRQ_EN(DMA_CH1);
}

void Dly_us()
{
    for (u16 i = 0; i < 180; i++)
        asm("nop");
}

void ccd_init()
{
    int i;
    //     初始化引脚
    // CLK PTE2
    // SI  PTE3
    // AO  AD9
    gpio_init(PORTE, 2, GPO, 0);
    gpio_init(PORTE, 3, GPO, 0);
    ADC_Init(ADC0);        // AO   所在的AD通道的管脚号
                           //   曝光部分
    gpio_set(PORTE, 3, 1); //SI = 1
    gpio_set(PORTE, 2, 1); //CLK = 1
    Dly_us();
    gpio_set(PORTE, 3, 0); //SI = 0
    gpio_set(PORTE, 2, 0); //CLK = 0
    Dly_us();
    for (i = 0; i < 256; i++) {
        gpio_turn(PORTE, 2); //产生128个CLK时钟,曝光时序
        Dly_us();
    }
}

void ccd_upload()
{
    u8 i;
    uart_putchar(UART4, 0xFF); //图像头
    for (i = 0; i < 128; i++) {
        if (((CCD[i] >> 8) & 0xFF) > 0) //高位有、数值
            uart_putchar(UART4, 0x7F); //输出最高值127
        else //只有低8位有数据
            uart_putchar(UART4, CCD[i] >> 1 & 0xFF); //直接输出低位值的一半//>>2
    }
}
void ccd_gather()
{
    //  128个像素点采集程序
    // CLK PTE2
    // SI  PTE3
    // AO  AD9
    u8 index = 127, i = 0;
    gpio_set(PORTE, 3, 1); //SI = 1
    gpio_set(PORTE, 2, 1); //CLK = 1
    Dly_us();
    gpio_set(PORTE, 3, 0); //SI = 0
    for (i = 0; i < 8; i++)
        Dly_us();
    gpio_set(PORTE, 2, 0); //CLK = 0
    Dly_us();

    CCD[index] = ADC_Ave(ADC0, ADC0_DM1, ADC_12bit, 2);
    for (i = 0; i < 254; i++) // i<254,是之前已经采集了一个像素点，这里只需采集127个
    {
        gpio_turn(PORTE, 2); //产生128个CLK时钟
        Dly_us();
        if ((i % 2) == 1) //  此时CLK = 1，为高电平
        {
            CCD[--index] = ADC_Ave(ADC0, ADC0_DM1, ADC_12bit, 2);
        }
    }
    gpio_set(PORTE, 2, 1); // 产生最后一个CLK时钟
    Dly_us();
    gpio_set(PORTE, 2, 0); // 产生最后一个CLK时钟
    Dly_us();
}
void ccd_proc()
{
    u8 i;
    u16 sum;
    ccd_max_light = 0;
    ccd_light_width = 0;
    u8 ccd_light_centre1 = 0;
    u8 ccd_light_centre2 = 0;
    ccd_light_centre = 0;
    ccd_light_average = 0;
    for (i = 0; i < 128; i++) {
        ccd_light_average += CCD[i];
        if (CCD[i] > ccd_light_threshorld) {
            ccd_light_width++;
        }
        if (CCD[i] > ccd_max_light) {
            ccd_max_light = CCD[i];
            ccd_light_centre1 = i;
        }
        if (CCD[i] >= ccd_max_light) {
            ccd_light_centre2 = i;
        }
    }
    if (ccd_light_width < 2) {
        ccd_light_width = 0;
        ccd_light_centre = 75;
    }
    ccd_light_average /= 128;
    ccd_light_average &= 0xFF;
    ccd_light_width &= 0xFF;
    ccd_light_centre = (ccd_light_centre1 + ccd_light_centre2) / 2;
    ccd_max_light &= 0xFF;
}

void DMAProc()
{
    u8 i = 0;
    tof_value = 0;
    while (uart_dma_recv[i++] != '\n' && i < sizeof(uart_dma_recv))
        ; //移动到起始标记\n后
    if (i >= sizeof(uart_dma_recv))
        return;                                                    //如果没有找到，退出
    while (uart_dma_recv[i] != 'm' && i < sizeof(uart_dma_recv)) { //处理数字段
        tof_value *= 10;
        tof_value += uart_dma_recv[i] - '0';
        i++;
    }
}

void decide_speed()
{
    u16 light_x_middle = 72;
    u8 ccd_light_middle = 64; //CCD光源在正前方方向时的横坐标

    servValue = servMotorNear; //默认收回挡板
    //默认转圈找光
    //转圈找光时，防止后方轮子撞到轮子
    if (!gpio_get(PORTB, 16)) last_direction = 1; //左边检测到障碍物，屁股向右
    if (!gpio_get(PORTB, 17)) last_direction = 0; //右边检测到障碍物，屁股向左
    g_MotorPWM[1-ccd_last_direction] = pwm_rotate[0];
    g_MotorPWM[ccd_last_direction] = pwm_rotate[1];
    // 寻光
    // CMOS优先
    double speed_up_rate = (30 > ccd_light_width) ? (30 - ccd_light_width)/30.0 : 0;//这是无敌逍哥的代码
    //double speed_up_rate =  (30.0-0.3*ccd_light_width)/30.0 ;
    
    u16 pwm_temp = speed_up_rate * (pwm_max-pwm_near) + pwm_near;//这是无敌逍哥的代码
    //u16 pwm_temp = speed_up_rate ;
    if (ccd_light_width >= 2) { //其次用线阵CCD判断距离 //原来是>=2
    
       // g_MotorPWM[left] =   6000*(50.0-0.9*ccd_light_width)/50.0  + 0.15*((ccd_light_centre-64.0)/3.0)*((ccd_light_centre-64.0)/3.0)*((ccd_light_centre-64.0)/3.0);//出现了在刚找到光的时候会停车的问题。
        //g_MotorPWM[right] =  6000*(50.0-0.9*ccd_light_width)/50.0  - 0.15*((ccd_light_centre-64.0)/3.0)*((ccd_light_centre-64.0)/3.0)*((ccd_light_centre-64.0)/3.0);
    
      
        g_MotorPWM[left] = pwm_temp * (1.0 - (128 - ccd_light_centre) * 0.4 / 128);
        g_MotorPWM[right] = pwm_temp * (1.0 - (ccd_light_centre) * 0.4 / 128);    //这是无敌逍哥的代码
        ccd_last_direction = ccd_light_centre > ccd_light_middle;
    }
    // 如果距离接近，停车50ms后进入灭灯模式
    if (distance[0] <= near_distance_range-20 || distance[1] <= near_distance_range || distance[2] <= near_distance_range-20) {
        if (!near_flag) { //如果第一次进入灭灯模式，先停车200ms
            g_MotorBrake[left] = 1;
            g_MotorBrake[right] = 1;
            brake_start_time = TimeCount;
            near_flag = 1;
        }
    }
    if ((TimeCount - brake_start_time + TimeCount_Max) % TimeCount_Max >= 200) { //200ms后关闭刹车，该干嘛干嘛
            g_MotorBrake[left] = 0;
            g_MotorBrake[right] = 0;
        }
    // 灭灯模式下的动作
    if (near_flag) {
        servValue = servMotorFar;   //伸出挡板
        if (ccd_light_width >= 2) { //其次用线阵CCD判断距离
            g_MotorPWM[left] = ccd_light_centre * 1.0 / (128) * pwm_min;
            g_MotorPWM[right] = pwm_min - g_MotorPWM[left];
        }
        if (ccd_light_width <= 45) {
            //距离灯还有一定距离，需要避障
            if (distance[0] < distance[2]) {
                //障碍物在左边，右转
                g_MotorPWM[right] *= 0.8;
            }
            if (distance[0] > distance[2]) {
                //障碍物在右边，左转
                g_MotorPWM[left] *= 0.8;
            }
        }
        if (ccd_light_width <= 15) //已经灭灯
        {
            //如果刹车，先把刹车退了
            g_MotorBrake[left] = 0;
            g_MotorBrake[right] = 0;
            //倒车
            back_start_time = TimeCount;
            near_flag = 0;
            move_back_direction = distance[0] > distance[1]; //左0右1
            move_back_flag = 1;
            g_MotorPWM[left] = pwm_back;
            g_MotorPWM[right] = pwm_back;

        }
    }
    // 倒车，持续时间都是0.8s
    if (move_back_flag) {
        g_MotorPWM[left] = -pwm_back;
        g_MotorPWM[right] = -pwm_back;
        g_MotorPWM[move_back_direction] = g_MotorPWM[1-move_back_direction]* 1.5;
        if (!gpio_get(PORTA, 12) || !gpio_get(PORTA, 13)){
            //倒车时后方对管检测到障碍物，停止倒车
            move_back_flag = 0;
        }
        if ((TimeCount - back_start_time + TimeCount_Max) % TimeCount_Max > 800) {
            //已经倒车，需要回到正常状态
            move_back_flag = 0;
        }
    }
    FTM_PWM_Duty(FTM2, CH1, servValue); //挡板状态更新
}

void image_erode(unsigned char (*t)[cmoscol], unsigned char (*temp)[cmoscol], unsigned char b)
{
    u8 a = b;
    unsigned int i, j;
    u8 V = cmoscol, H = cmosrow;
    for (i = 1; i < V - 1; i++)
        for (j = 1; j < H - 1; j++)
            if (t[i][j - 1] == a || t[i][j + 1] == a || t[i - 1][j] == a || t[i + 1][j] == a) //对图像内部进行置一处理
                temp[i][j] = a;
    for (i = 1; i < V - 1; i++) {
        if (t[i - 1][0] == a || t[i + 1][0] == a) //对图像第一列数据置一处理
            temp[i][0] = a;
        if (t[i - 1][H - 1] == a || t[i + 1][H - 1] == a) //对图像最后一列置一处理
            temp[i][H - 1] = a;
    }
    for (j = 0; j < H - 1; j++) {
        if (t[0][j - 1] == a || t[0][j + 1] == a) //对第一行进行置一处理
            temp[0][j] = a;
        if (t[V - 1][j - 1] == a || t[V - 1][j + 1] == a) //对最后一行进行置一处理
            temp[V - 1][j] = a;
    }
    if (t[0][1] == a || t[1][0] == a) //对四个角进行置一处理
        temp[0][0] = a;
    if (t[0][H - 2] == a || t[1][H - 1] == a)
        temp[0][H - 1] = a;
    if (t[V - 2][0] == a || t[V - 1][1] == a)
        temp[V - 1][0] = a;
    if (t[V - 2][H - 1] == a || t[V - 1][H - 2] == a)
        temp[V - 1][H - 1] = a;
}

