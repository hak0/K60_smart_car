//TODO:正式运行时将options-compiler中的optimization开高
//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///           2016-4-10               ///
//////////////////////////////////////////
// 940 1120 1300
#include "common.h"
#include "include.h"

#define tof_value_min 800 //最小距离值，再小就触发灭灯

#define servMotorCenture 1160 //880 //920 //960 //1710     //舵机中心位置
#define servMotorNear 1160    //730 //770  //1580
//舵机左极限，很关键，限幅防止舵机打死
#define servMotorFar 550 //1030 //1070 //1050 //1840
//舵机右极限，很关键，限s幅防止舵机打死

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

u16 servPram; //舵机转角比例系数 放大10倍
u16 dPram;    //舵机转角积分系数  放大100倍

u16 servPram1 = 12; //22;//22;     //直线时 舵机转角比例系数 放大10倍
u16 dPram1 = 16;    //12;//12;        //直线时  舵机转角积分系数  放大100倍

u16 servPram2 = 14; //22;//22;     //小弯时 舵机转角比例系数 放大10倍
u16 dPram2 = 13;    //12;//12;        //小弯时  舵机转角积分系数  放大100倍

u16 servPram3 = 19; //22;//22;     //大弯时 舵机转角比例系数 放大10倍
u16 dPram3 = 10;    //12;//12;        //大弯时  舵机转角积分系数  放大100倍

u8 Buffer1[ROW][COL] = { 0 }; //37*8=296
/* 通过DMA采集(在isr.c中行中断中触发DMA采集)二维图像存储于Buffer1数组中，
 * ROW,COL在Include.h中定义 */

u8 Threshold[40];

u8 cmos[cmosrow][cmoscol] = { 0 }; //进行处理的数据

s32 zuoPos = 0; //左边界位置[有符号数]
s32 youPos = 0; //右边界位置[有符号数]

s32 PreErr = 0, LastErr = 0, dErr = 0;
u32 centure = 0;    //计算出的当前车体的中心位置[无符号数]
s32 centureErr = 0; //计算出的当前车体位置偏差 [有符号数]
s32 turn1 = 0;      //边界走势  正为向左偏转  负为向右偏转
s32 turn2 = 0;
u8 zuoEdgeFlag = 0; //左边界确认标志，找到后边界置位
u8 youEdgeFlag = 0; //右边界确认标志，找到后边界置位

u8 VSYN_Flag = 0; //完成一场标识位  0为未完成；1为完成
u8 time;
u8 ensend; //允许发送
u8 enpwm;
u8 entof;
u8 speedmodi; //速度修改标志

// 光源位置
u16 light_x;
u16 light_y;
u8 see_light;

// 读取对管
u8 get_dual_diode();

//
u16 left_lost = 0, right_lost = 0, black_left, black_right, lost_flag,
    left_black, right_black, flag1 = 0;
s16 sum_m = 0, part2_sum = 0, field_lost = 0, break_sum = 0;
u8 qian2 = 0, shiziwan_left = 0, shiziwan_right = 0;
//
unsigned short DuoCenter; //舵机中间值 120HZ
s32 TimeCount = 0;        //1ms中断标志
s16 g_nMotorPulse[2];
s32 g_nMotorPulseSigma[2];
s16 g_MotorPWM[2] = { 0, 0 }; // 左右电机PWM值
u8 g_MotorBrake[2];           // 左右电机刹车信号
s16 speed[2] = { 0, 0 };      // 左右电机设置速度值

char tof_receive[32];
u8 tof_num_flag = 0;
u8 tof_index = 0;
u16 tof_value = 0; //tof测得的距离值

extern u8 TIME1flag_100ms, flag_1ms;

volatile u32 rowCnt = 0; //行计数

u8 encoder_dir;        //编码器方向
u8 encoder_select = 1; //选择编码器,0表示左1表示右

u32 brake_start_time = 0;
u32 back_start_time = 0;
u8 move_back_flag = 0;

//-----------------函数声明------------------
void ImageProc();
void GPIO_Init();
void PWM_Init();
void UART_Init();
void EXTI_Init();
void Image_Init();
void run();
void turn_off_light();
void update_pwm();
void CountThreshole(void);
void datatrans();
void tof_dma_init();
void PID();
void TOFProc();
void decide_speed();
//-----------------主函数--------------------
void main()
{
    u8 sccb_ack;
    DuoCenter = servMotorCenture; //舵机位置回到中心

    //debug标志
    ensend = 1;
    entof = 0;

    //开始初始化
    DisableInterrupts; //关闭中断开始初始化数据
    /* FTM1_QUAD_Iint();  //正交解码测速  A相---PTA12  B相---PTA13 */
    //但是我们的编码器并不是AB相的，需要从GPIO读入方向，从PTB16
    GPIO_Init(); //------GPIO初始化  用于指示SCCB状态
    sccb_init(); //-----SCCB初始化，用于配置摄像头状态
    sccb_ack = sccb_refresh();

    PWM_Init();  //-----PWM初始化，用于初始化舵机和电机的输出
    EXTI_Init(); //-----外部中断初始化，用于摄像头中断触发
    UART_Init(); //-----UART初始化，用于初始化串口
    //TODO:分别检测PIT有无时，PIT和摄像头中断的时间情况（用示波器）
    pit_init_ms(PIT0, 10); //初始化PIT0，定时时间为： 10ms
    //pit_init_ms(PIT1, 100);//初始化PIT1，定时时间为： 100ms
    Image_Init(); //----图像数组初始化
    EnableInterrupts;
    // 初始化结束

    uart_irq_EN(UART4);
    //uart_irq_EN(UART3);
    uart_putchar(UART4, 'S');
    uart_putchar(UART4, 't');
    uart_putchar(UART4, 'a');
    uart_putchar(UART4, 'r');
    uart_putchar(UART4, 't');
    uart_putchar(UART4, '.');
    uart_putchar(UART4, '.');

    rowCnt = 0;
    while (1) {
        if (VSYN_Flag == 1) //完成一幅图像采集
        {
            ImageProc();
            VSYN_Flag = 0;
            EnableInterrupts;
        }
        //TOFProc(); //TOF数据采集 //TODO:检查TOF采集对摄像头和定时器中断的影响，影响很大，本来7ms变成了400ms
        tof_value = 1400;
        if (entof == 1) {
            uart_putchar(UART4, tof_value / 1000 + '0');     //tof data
            uart_putchar(UART4, tof_value / 100 % 10 + '0'); //tof_data
            uart_putchar(UART4, tof_value / 10 % 10 + '0');  //tof_data
            uart_putchar(UART4, tof_value % 10 + '0');       //tof_data
            uart_putchar(UART4, '\n');                       //tof_data
        }
        // TODO：写一个大一统的decide_speed函数，把对管距离之类的都放在里面检测，每两场处理一次
        if (get_dual_diode() || (g_MotorBrake[left] == 1 && g_MotorBrake[right] == 1))
            turn_off_light();
        decide_speed(); //根据光源位置状态决定PWM值
        update_pwm();   //更新输出到驱动板的pwm值
    }
}

void GPIO_Init()
{
    gpio_init(PORTB, 16, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受编码器方向
    gpio_init(PORTB, 17, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受编码器方向
    /* gpio_init(PORTE, 12, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻 */
    //对管
    gpio_init(PORTE, 2, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻
    gpio_init(PORTE, 3, GPI_UP_PF, 1); //初始化PTB16为电平输入，上拉，接受光敏电阻
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
    FTM_PWM_init(FTM2, CH1, 50, DuoCenter); //初始化FTM0_CH0输出频率为50HZ,占空比为DuoCenter ：舵机 FTM2_CH1对应PTB19口
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

void CountThreshold(void) //找出黑点最大和白点最小的两个数，计算i行的阈值，就第0行用到了
{
    u16 i, j;
    unsigned char max1, min1;
    for (i = 30; i < 70; i++) {
        max1 = cmos[i][0];
        min1 = cmos[i][0];
        for (j = 1; j < 299; j++) //BLACKMAX==16,WHITEMAX==160
        {
            if ((cmos[i][j] < 250) && (cmos[i][j] > max1))
                max1 = cmos[i][j];
            if ((cmos[i][j] > 20) && (cmos[i][j] < min1))
                min1 = cmos[i][j];
        }
        Threshold[i] = (max1 + min1) / 2 + 10;
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
    see_light = (cnt > 10) ? 1 : 0;
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
        uart_putchar(UART4, see_light);
        uart_putchar(UART4, light_x >> 8);
        uart_putchar(UART4, light_x & 0xFF);
        uart_putchar(UART4, light_y & 0xFF);
    }
}

extern u8 tof_num_flag;
void run()
{
    gpio_turn(PORTE, 12); //PTE12取反，用于观察中断进入的时间间隔
#if 0                     //测速相关
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
void turn_off_light()
{
    /* 快把浴霸关上!(误) */
    /* 伸出挡板，挡光 */
    if (g_MotorBrake[left] == 0 && g_MotorBrake[right] == 0) {
        brake_start_time = TimeCount;
        g_MotorBrake[left] = 1;  //PWM置零
        g_MotorBrake[right] = 1; //刹车标记始能
        gpio_set(PORTE, 11, 1);  //物理刹车
    } else if (g_MotorBrake[left] == 1 && g_MotorBrake[right] == 1) {
        if ((TimeCount - brake_start_time) % 1000 >= 5)
            gpio_set(PORTE, 11, 0); //收回物理刹车
        if ((TimeCount - brake_start_time) % 1000 >= 20) {
            g_MotorBrake[left] = 0;
            g_MotorBrake[right] = 0;
            if (1) {
                move_back_flag = 1;
                back_start_time = TimeCount;
            } //如果没有光了，就开始后退
        }
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

void TOFProc()
{
    u8 i = 0;
    tof_value = 0;
    while (tof_receive[i++] != '\n' && i < sizeof(tof_receive))
        ; //移动到起始标记\n后
    if (i >= sizeof(tof_receive))
        return;                                                //如果没有找到，退出
    while (tof_receive[i] != 'm' && i < sizeof(tof_receive)) { //处理数字段
        tof_value *= 10;
        tof_value += tof_receive[i] - '0';
        i++;
    }
    tof_num_flag = 1;
}
void tof_dma_init()
{
    //dma对应串口UART3的dma配置,dma为CH1
    /* DMA_PORTx2BUFF_Init(DMA_CH3, (void*)&USART_TX_BUF, tof_receive_buf, )  */
    //SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
    DMAMUX_CHCFG1 = (1 << 7) | DMA_UART3_Rx; //启用DMA，触发通道位UART3
    //SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;
    //DMA_CR = 0;
    DMA_TCD1_SADDR = (unsigned long)&UART3_D;            //DMA源地址
    DMA_TCD1_DADDR = (unsigned long)&tof_receive[0]; //DMA目的地址
    DMA_TCD1_NBYTES_MLNO = 1;
    DMA_TCD1_ATTR = 0;     //8位传送
    DMA_TCD1_SOFF = 0;     //每次操作完源地址，源地址不增加
    DMA_TCD1_DOFF = 1;     //每次操作完目标地址，目标地址增加1
    DMA_TCD1_SLAST = 0;    //DMA完成一次输出之后即major_loop衰减完之后不更改源地址
    DMA_TCD1_DLASTSGA = 0; //DMA完成一次输出之后即major_loop衰减完之后不更改目标地址
    //每次读100个数据然后触发DMA中断
    DMA_TCD1_CITER_ELINKNO = 16; //计数初始值
    DMA_TCD1_BITER_ELINKNO = 16; //计数复位值
    DMA_TCD1_CSR = 0;             //该寄存器清零，做后续配置
    /* DMA_TCD1_CSR &= ~DMA_CSR_INTMAJOR_MASK; //“当计数到0后，发起DMA中断”功能关闭 */
    DMA_TCD1_CSR |= DMA_CSR_INTMAJOR_MASK; //“当计数到0后，发起DMA中断”功能开启
    
    DMA_TCD1_CSR |= DMA_CSR_DREQ_MASK; //“当计数到0后，自动清除该DMA请求标志”功能启动
    DMA_ERQ |= (1 << 1); //启动
}

u8 get_dual_diode()
{
    return (!gpio_get(PORTB, 16) || !gpio_get(PORTB, 17) || !gpio_get(PORTE, 2) || !gpio_get(PORTE, 3));
}
void decide_speed()
{
    const u16 light_x_middle = 72;
    const u16 pwm_max = 3000;
    const u16 pwm_near = 1500;
    const u16 pwm_min = 1000;
    const u16 pwm_rotate[2] = { 4000, 0 };
    /* const u16 tof_value_threshold = 800; */
    const u16 tof_value_threshold = 400;

    DuoCenter = servMotorNear; //默认收回挡板
    //三种情况
    //1. 摄像头未捕获，TOF无反馈 -> 打转一段时间后寻光
    //2. 摄像头未捕获，TOF反馈 -> 直行，根据TOF调速
    //3. 摄像头捕获，TOF无反馈 -> 确定方向，前进
    //4. 摄像头捕获，TOF反馈 -> 避障,向一侧转向,在TOF无反馈的基础上调整
    //三个环节：
    //倒车标志的处理，寻光(特殊情况)
    //整体速度的把握，摄像头优先级高于TOF，有障碍的情况再议
    //转向如果有摄像头指导，按照摄像头的数据来
    //
    //特殊情况
    if (move_back_flag) {
        //倒退1s，不接收反驳
        if ((TimeCount - back_start_time) % 1000 >= 20) {
            move_back_flag = 0;
        }
        g_MotorPWM[left] = pwm_max * -0.6;
        g_MotorPWM[right] = pwm_max * -0.6;
        return;
    }
    if (tof_value >= tof_value_threshold && !see_light) {
        // 打转找光,800ms打转,200ms前进
        if (TimeCount <= 800) {
            g_MotorPWM[left] = pwm_rotate[left];
            g_MotorPWM[right] = pwm_rotate[right];
        } else {
            g_MotorPWM[left] = pwm_max * 0.6;
            g_MotorPWM[right] = pwm_max * 0.6;
        }
        DuoCenter = servMotorNear; //收回挡板
    }
    // 速度整体把握
    if (see_light) {
        /* if (gpio_get(PORTE, 12)) 可以用光敏电阻区分是障碍物还是光源*/
        //TODO:区分光源与障碍物，避障
        /* 这里都默认为光源 */
        // 向光源移动，速度随光源远近调整
        g_MotorPWM[left] = (pwm_max - pwm_min) * (62 - light_y) / 62 + pwm_min;
        g_MotorPWM[right] = (pwm_max - pwm_min) * (62 - light_y) / 62 + pwm_min;
    } else if (tof_value < tof_value_threshold) { //第一优先级：TOF,顺便伸出挡板
        g_MotorPWM[left] = (tof_value - tof_value_min) * (pwm_near - pwm_min) / (tof_value_threshold - tof_value_min) + pwm_min;
        g_MotorPWM[right] = (tof_value - tof_value_min) * (pwm_near - pwm_min) / (tof_value_threshold - tof_value_min) + pwm_min;
    }
    //转向调节
    if (light_x <= light_x_middle - 3) {
        g_MotorPWM[left] -= 1200;
    } else if (light_x >= light_x_middle + 3) {
        g_MotorPWM[right] -= 1200;
    }
    //是否伸出灭灯板
    if (tof_value <= tof_value_threshold)
        DuoCenter = servMotorFar; //伸出挡板
    FTM_PWM_Duty(FTM2, CH1, DuoCenter);
}

