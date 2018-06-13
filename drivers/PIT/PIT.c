/******************** (C) COPYRIGHT 2012-2013 �Ĭ�Ƽ� DEMOK*******************
 * �ļ���         ��PIT.c
 * ����           ��PIT�����жϼ�ʱ������ʱ�ж�

 * ʵ��ƽ̨       �� �Ĭ�Ƽ�DEMOK Kinetis������
 * ����           �� �Ĭ�Ƽ�DEMOK Kinetis����С��

 * �Ա�����       �� http://shop60443799.taobao.com/
 * ������������   �� 1030923155@qq.com 
 * ��������QQȺ   �� 103360642

 * ����޶�ʱ��    ��2012-11-5
 * �޶�����        ����
**********************************************************************************/


#include "common.h"
#include  "PIT.h"     //�����жϼ�ʱ��


/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�pit_init
*  ����˵����PITn��ʱ�ж�
*  ����˵����PITn        ģ��ţ�PIT0~PIT3��
             cnt         ��ʱʱ��(��λΪbusʱ������)
*  �������أ���
*
*************************************************************************/
void pit_init(PITn pitn, u32 cnt)
{
    //PIT �õ��� Bus Clock ����Ƶ��
    //������� = ����Ƶ�� * ʱ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //ʹ��PITʱ��

    /* PITģ����� PIT Module Control Register (PIT_MCR) */
    PIT_MCR         &= ~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );      //ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������

    /* ��ʱ������ֵ���� Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt;                                          //��������ж�ʱ��

    //��ʱʱ�䵽�˺�TIF �� 1 ��д1��ʱ��ͻ���0
    PIT_Flag_Clear(pitn);                                             //���жϱ�־λ

    /* ��ʱ�����ƼĴ��� Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= ( PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK );   //ʹ�� PITn��ʱ��,����PITn�ж�

    enable_irq(pitn + 68);			                                //���������ŵ�IRQ�ж�
}


/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�pit_init
*  ����˵����PITn��ʱ�ж�
*  ����˵����PITn        ģ��ţ�PIT0~PIT3��
             cnt         ��ʱʱ��(��λΪbusʱ������)
*  �������أ���
*
*************************************************************************/
void pit_dma_init(PITn pitn, u32 cnt)
{
    //PIT �õ��� Bus Clock ����Ƶ��
    //������� = ����Ƶ�� * ʱ��

    /* ����ʱ��*/
    SIM_SCGC6       |= SIM_SCGC6_PIT_MASK;                            //ʹ��PITʱ��

    /* PITģ����� PIT Module Control Register (PIT_MCR) */
    //PIT_MCR         &=~(PIT_MCR_MDIS_MASK | PIT_MCR_FRZ_MASK );       //ʹ��PIT��ʱ��ʱ�� ������ģʽ�¼�������
    PIT_MCR = 0;

    /* ��ʱ������ֵ���� Timer Load Value Register (PIT_LDVALn) */
    PIT_LDVAL(pitn)  = cnt;                                          //��������ж�ʱ��

    //��ʱʱ�䵽�˺�TIF �� 1 ��д1��ʱ��ͻ���0
    PIT_Flag_Clear(pitn);                                             //���жϱ�־λ

    /* ��ʱ�����ƼĴ��� Timer Control Register (PIT_TCTRL0) */
    PIT_TCTRL(pitn) |= (0
                        | PIT_TCTRL_TEN_MASK  //ʹ�� PITn��ʱ��
                        | PIT_TCTRL_TIE_MASK  //��PITn�ж�
                       );

    enable_irq(pitn + 68);			                                //���������ŵ�IRQ�ж�
}