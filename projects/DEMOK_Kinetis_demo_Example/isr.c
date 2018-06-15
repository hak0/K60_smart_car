//////////////////////////////////////////
////         ���ݺ���ΰҵ      ///////////
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
extern u16 tof_value;            //tof��õľ���
extern unsigned short DuoCenter; ////����м�ֵ 120HZ
extern unsigned short dianjispeed;
extern u8 ensend; //������

extern u16 servPram;
extern u16 dPram;
extern u8 enpwm;
extern u8 speedmodi;

extern void run();

extern u32 rowCnt; //�м���
extern u8 Buffer1[ROW][COL];
u8 SampleFlag = 0;
extern u8 VSYN_Flag;

unsigned char flag_1ms = 0;
u8 TIME1flag_100ms = 0;
/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�USART4_IRQHandler
*  ����˵��������5 �ж� ���� ������
*  ����˵������
*  �������أ���
*
*************************************************************************/
void USART4_IRQHandler(void)
{
    uint8 ch;
    //DisableInterrupts;		    //�����ж�
    //����һ���ֽ����ݲ��ط�
    ch = uart_getchar(UART4); //���յ�һ������
    data_receive[index_bt] = ch;
    index_bt++;
    if (data_receive[0] == '#') //��ʶͷ
    {
        if (ch == '$') {
            if (data_receive[1] == 'D') //�����ֵ
            {
                DuoCenter = (data_receive[2] - 48) * 1000 + (data_receive[3] - 48) * 100 + (data_receive[4] - 48) * 10 + (data_receive[5] - 48);
                FTM_PWM_Duty(FTM2, CH1, DuoCenter);
            }
            if (data_receive[1] == 'Y') // ��������ͷ�ϴ�
            {
                ensend = 1;
            }
            if (data_receive[1] == 'N') // ��ֹ����ͷ�ϴ�
            {
                ensend = 0;
            }
            if (data_receive[1] == 'X') // ����PWMֵ�ϴ�
            {
                enpwm = 1;
            }
            if (data_receive[1] == 'Z') // ��ֹPWMֵ�ϴ�
            {
                enpwm = 0;
            }
            if (data_receive[1] == 'P') //�������ϵ��
            {
                servPram = (data_receive[2] - 48) * 100 + (data_receive[3] - 48) * 10 + (data_receive[4] - 48);
            }
            if (data_receive[1] == 'F') //���΢��ϵ��
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
    //EnableInterrupts;		    //�����ж�
    //uart_putchar(UART0,'A');
}

void USART3_IRQHandler(void)
{
    //TODO:���õ����޸�TOF���ã������Ƶ�ʷ���
    uint8 ch;
    ch = uart_getchar(UART3);
    if (ch == '\n') { // ��ʼ��ǣ���ʼ������յ�����
        for (index_tof = 0; index_tof < 5; index_tof++) {
            tof_receive[index_tof] = 0x00; //����ռ�¼�������ݵ�����
        }
        index_tof = 0;      //�±��λ
        tof_num_flag = 1;   //��ʾ�����������������֣�����
    } else if ((ch == 'm') && (tof_num_flag == 1)) { //�������
        tof_num_flag = 0;   //��ʾ����������������ĸ�ͻ��з�������
        tof_value = 0;
        for (; index_tof >= 0; index_tof--) { //�������е�ֵת��Ϊ���ֲ����浽��Ӧ����
            tof_value += tof_receive[index_tof];
            tof_value *= 10;
            index_tof--;
        }
    } else if (tof_num_flag) { //������յ����ݵ�����
        tof_receive[index_tof] = ch - '0';
        index_tof++;
    }
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�PIT0_IRQHandler
*  ����˵����PIT0 ��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
#if 1
void PIT0_IRQHandler(void) //1ms
{
    // LED_turn(LED1);             //LED1��ת
    PIT_Flag_Clear(PIT0); //���жϱ�־λ
    flag_1ms = 1;         //1ms�жϱ�־
    run();                //���ٺ���
}
#else
void PIT0_IRQHandler(void)
{
    PIT_Flag_Clear(PIT0); //���жϱ�־λ
}
#endif

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�SysTick_Handler
*  ����˵����ϵͳ�δ�ʱ���жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void SysTick_Handler(void)
{
    //    OSIntEnter();
    //    OSTimeTick();
    //    OSIntExit();
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void HardFault_Handler(void)
{
    while (1) {
        printf("\n****Ӳ���Ϸô���!!!*****\r\n\n");
    }
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�PendSV_Handler
*  ����˵����PendSV��������ϵͳ���ã��жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void PendSV_Handler(void)
{
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�PORTE_IRQHandler
*  ����˵����PORTE�˿��жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*
*************************************************************************/

void PORTE_IRQHandler()
{
    unsigned int lie;
    //---VSYN���жϴ���
    if (PORTE_ISFR & (1 << 1)) //PTE1�����ж�  ���ж�
    {
        PORTE_ISFR |= (1 << 1); //д1���жϱ�־λ
        rowCnt = 0;
        SampleFlag = 1;
    }
    //---HREF���жϴ���
    if (PORTE_ISFR & (1 << 0)) //PTE0�����ж�
    {
        PORTE_ISFR |= (1 << 0); //д1���жϱ�־λ
        if (SampleFlag == 0)    //����һ��ʱ����
        {
            return;
        }
        //-------------DMA��ʼ��ͨ��4������ԴΪPTD��ÿ�δ�������ImageBuf[]ָ���У�PCLK��PTA19������ÿ�δ���1���ֽڣ�ÿ�δ�������300�Σ������ش���
        else {
            if (rowCnt > 10) {
                for (lie = 0; lie < 70; lie++) //130  ��ʱ�޸ģ����Ե���ͼ�������
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
*                            �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�DMA_CH4_Handler
*  ����˵����DMAͨ��4���жϷ�����
*  ����˵������
*  �������أ���
*
*************************************************************************/
void DMA_CH4_Handler(void)
{
    DMA_IRQ_CLEAN(DMA_CH4); //���ͨ�������жϱ�־λ    (���������ٴν����ж�)
    DMA_IRQ_DIS(DMA_CH4);   //��ֹDMA   �ȴ���һ�����ж����ٿ���DMA
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM0_IRQHandler
*  ����˵����FTM0���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/
void FTM0_IRQHandler()
{
}
/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM2_IRQHandler
*  ����˵����FTM2���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/
void FTM2_IRQHandler()
{
}
/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�FTM1_IRQHandler
*  ����˵����FTM1���벶׽�жϷ�����
*  ����˵������
*  �������أ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸ģ��ο����еĴ�������Լ��Ĺ���
*
*************************************************************************/
void FTM1_IRQHandler()
{
    u8 s = FTM1_STATUS; //��ȡ��׽�ͱȽ�״̬  All CHnF bits can be checked using only one read of STATUS.
    u8 n;
    FTM1_STATUS = 0x00; //���жϱ�־λ

    n = 0;
    if (s & (1 << n)) {
        FTM_CnSC_REG(FTM1_BASE_PTR, n) &= ~FTM_CnSC_CHIE_MASK; //��ֹ���벶׽�ж�
        /*     �û�����       */
        //LED_turn(LED1);                             //��תLED1
        printf("\nFTM1�����ж�\n");

        /*********************/
        //�����������￪�����벶׽�жϣ�������main�����������Ҫ������
        //ͨ�� CH0��CH1��Ch2��Ch3 ���˲���
        //FTM_CnSC_REG(FTM1_BASE_PTR,n) |= FTM_CnSC_CHIE_MASK;  //�������벶׽�ж�
        //delayms(10);        //��Ϊ������ź�������̲��ȶ������״���������벶׽�����������ʱ
        //�����ǵ��жϲ�Ӧ�ù�����ʱ�����Կ����벶׽�жϾͷ���main�����������Ҫ������
    }

    n = 1;
    if (s & (1 << n)) {
        FTM_CnSC_REG(FTM1_BASE_PTR, n) &= ~FTM_CnSC_CHIE_MASK; //��ֹ���벶׽�ж�
        /*     �û�����       */

        /*********************/
        //�����������￪�����벶׽�ж�
        //FTM_CnSC_REG(FTM1_BASE_PTR,n) |= FTM_CnSC_CHIE_MASK;  //�������벶׽�ж�
    }
}

