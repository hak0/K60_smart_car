//////////////////////////////////////////
////         ���ݺ���ΰҵ      ///////////
///   http://shop36538723.taobao.com /////
///                                  ////
///     ��лҰ���ṩ�Ŀ��ļ�          ///
///           2015-4-10               ///
//////////////////////////////////////////

#include "common.h"
#include "gpio.h"
#include "exti.h"




/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�exti_init
*  ����˵����EXTI�ⲿGPIO�жϳ�ʼ��
*  ����˵����PORTx       �˿ںţ�PORTA,PORTB,PORTC,PORTD,PORTE��
*            n          �˿�����
*            exti_cfg   ����ѡ�����������ѡ��
*  �������أ���
*
*************************************************************************/
u32 debug_temp;

void  exti_init(PORTx portx, u8 n, exti_cfg cfg)
{
    SIM_SCGC5 |= (SIM_SCGC5_PORTE_MASK << portx);    //����PORTx�˿�

    PORT_PCR_REG(PORTX[portx], n) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(cfg & 0x7f ) | PORT_PCR_PE_MASK | ((cfg & 0x80 ) >> 7); // ����GPIO , ȷ������ģʽ ,������������������
    GPIO_PDDR_REG(GPIOx[portx]) &= ~(1 << n);       //����ģʽ
    debug_temp = portx + 87;
    enable_irq(portx + 87);                         //ʹ��PORT�жϣ�PORTA��ISR�жϺ�Ϊ87
    
}