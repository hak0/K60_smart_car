//////////////////////////////////////////
////         ���ݺ���ΰҵ      ///////////
///   http://shop36538723.taobao.com /////
///                                  ////
///     ��лҰ���ṩ�Ŀ��ļ�          ///
///           2015-4-10               ///
//////////////////////////////////////////

#include "common.h"
#include "gpio.h"
#include "PIT.h"
#include "dma.h"

//u8 counttempaddr;
#define COUNTSADDR   0x4004000C  //(&counttempaddr)
#define COUNTDADDR   0x4004000C  //(&counttempaddr)

u32 count_init[16];         //��������16��ͨ���ĳ�ʼ������ֵ




/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*  �������ƣ�DMA_PORTx2BUFF_Init
*  ����˵����DMA��ʼ������ȡ�˿����ݵ��ڴ�
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*            SADDR                Դ��ַ( (void * )&PTx_BYTEn_IN �� (void * )&PTx_WORDn_IN   )
*            DADDR                Ŀ�ĵ�ַ
*            PTxn                 �����˿�
*            DMA_BYTEn            ÿ��DMA�����ֽ���
*            count                һ����ѭ�������ֽ���
*            DMA_PORTx2BUFF_cfg   DMA��������
*  �������أ���
*
*************************************************************************/
void DMA_PORTx2BUFF_Init(DMA_CHn CHn, void *SADDR, void *DADDR, PTxn ptxn, DMA_BYTEn byten, u32 count, DMA_PORTx2BUFF_cfg cfg)
{
    u8 n, i, tmp;

    ASSERT(                                             //�ö��Լ��Դ��ַ��ÿ�δ����ֽ����Ƿ���ȷ
        (   (byten == DMA_BYTE1)                        //����һ���ֽ�
            && ( (SADDR >= &PTA_BYTE0_IN) && (SADDR <= ( &PTE_BYTE3_IN )))
        )

        || (   (byten == DMA_BYTE2)                   //���������ֽ�(ע�⣬���ܿ�˿�)
               && ( (SADDR >= &PTA_BYTE0_IN)
                    && (SADDR <= ( &PTE_WORD1_IN ))
                    && (((u32)SADDR & 0x03) != 0x03) )         //��֤����˿�
           )

        || (   (byten == DMA_BYTE4)                   //�����ĸ��ֽ�
               && ((SADDR >= &PTA_BYTE0_IN) && (SADDR <= ( &PTE_BYTE0_IN )))
               && (((u32)SADDR & 0x03) == 0x00)           //��֤����˿�
           )
    );

    u8 BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��

    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (u32)SADDR;                         // ����  Դ��ַ
    DMA_DADDR(CHn) =    (u32)DADDR;                         // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0x00u;                              // ����Դ��ַƫ�� = 0x0, ������
    DMA_DOFF(CHn)  =    BYTEs;                              // ÿ�δ����Ŀ�ĵ�ַ�� BYTEs

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������
    DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKYES_BITER(count);//��ʼ��ѭ������


    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    //��CR[EMLM] = 0 ʱ:
    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */


    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   0;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   (u32)( (cfg & 0x20) == 0 ? (-count)  : 0 ); //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            //| DMAMUX_CHCFG_TRIG_MASK                        /* Trigger Mode: Periodic   PIT���ڴ�������ģʽ   ͨ��1��ӦPIT1������ʹ��PIT1����������Ӧ��PIT��ʱ���� */
            | DMAMUX_CHCFG_SOURCE((ptxn >> 5) + DMA_Port_A) /* ͨ����������Դ:     */
                                             );

    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (ptxn>>5));                                                               //����PORTx�˿�
    GPIO_PDDR_REG(GPIOx[(ptxn>>5)]) &= ~(1 << (ptxn & 0x1f));                                                       //���ö˿ڷ���Ϊ����
    PORT_PCR_REG(PORTX[(ptxn>>5)], (ptxn & 0x1F)) = ( 0
            | PORT_PCR_MUX(1)               // ����GPIO
            | PORT_PCR_IRQC(cfg & 0x03 )    // ȷ������ģʽ
            | ((cfg & 0xc0 ) >> 6)          // �����������������裬����û��
                                                    );
    GPIO_PDDR_REG(GPIOx[(ptxn>>5)]) &= ~(1 << (ptxn && 0x1F));                                                      //����ģʽ

    /*  ��������Դ   */
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << ((((u32)SADDR) & 0x1ff)>>6));             //����PORTx�˿�
    switch(byten)
    {
    case DMA_BYTE1:
        *((u8 *)((u32)SADDR + 4)) = 0;   //����Ϊ���뷽��Ϊʲô��4��PDIR��ַ��4�󣬾ͱ�ɶ�Ӧ��PDDR��ַ
        break;
    case DMA_BYTE2:
        *((u16 *)((u32)SADDR + 4)) = 0;
        break;
    case DMA_BYTE4:
        *((u32 *)((u32)SADDR + 4)) = 0;
        break;
    default:
        assert_failed(__FILE__, __LINE__);
        break;
    }

    /*  ����Դ�ܽ�ѡ���ܽ�  */
    n = (u8)(((u32)SADDR - ((u32)(&PTA_BYTE0_IN))) & 0x3f);         //��С�����ź�
    tmp = n + (BYTEs << 3);                                         //�������ź�
    for(i = n; i < tmp; i++)
    {
        PORT_PCR_REG(PORTX[   ((((u32)SADDR)&0x1ff)>>6)    ], i) = (0
                | PORT_PCR_MUX(1)
                | GPI_DOWN             //����ԴӦ��������Ĭ�϶�ȡ������0
                                                                   );
    }

   //-----------------------------------------------------------------
    DMA_IRQ_EN(CHn);                                //����DMAͨ������
    DMA_DIS(CHn);                                   // ��ֹDMA
}

/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*
*  �������ƣ�DMA_count_Init
*  ����˵����DMA�ۼӼ�����ʼ��
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*            PTxn                 �����˿�
*            count                �ۼӼ����ж�ֵ
*            DMA_Count_cfg        DMA��������
*  �������أ���
*
*************************************************************************/
void DMA_count_Init(DMA_CHn CHn, PTxn ptxn, u32 count, DMA_Count_cfg cfg)
{
    u8 byten = DMA_BYTE1;
    u8 BYTEs = (byten == DMA_BYTE1 ? 1 : (byten == DMA_BYTE2 ? 2 : (byten == DMA_BYTE4 ? 4 : 16 ) ) ); //���㴫���ֽ���
    if(count > 0x7FFF )count = 0x7FFF;
    count_init[CHn] = count;

    /* ����ʱ�� */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;                        //��DMAģ��ʱ��
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;                     //��DMA��·������ʱ��

    /* ���� DMA ͨ�� �� ������ƿ� TCD ( Transfer Control Descriptor ) */
    DMA_SADDR(CHn) =    (u32)COUNTSADDR;                    // ����  Դ��ַ
    DMA_DADDR(CHn) =    (u32)COUNTDADDR;                    // ����Ŀ�ĵ�ַ
    DMA_SOFF(CHn)  =    0;                                  // ����Դ��ַ����
    DMA_DOFF(CHn)  =    0;                                  // ÿ�δ����Ŀ�ĵ�ַ����

    DMA_ATTR(CHn)  =    (0
                         | DMA_ATTR_SMOD(0x0)                // Դ��ַģ����ֹ  Source address modulo feature is disabled
                         | DMA_ATTR_SSIZE(byten)             // Դ����λ�� ��DMA_BYTEn  ��    SSIZE = 0 -> 8-bit ��SSIZE = 1 -> 16-bit ��SSIZE = 2 -> 32-bit ��SSIZE = 4 -> 16-byte
                         | DMA_ATTR_DMOD(0x0)                // Ŀ���ַģ����ֹ
                         | DMA_ATTR_DSIZE(byten)             // Ŀ������λ�� ��DMA_BYTEn  ��  ���òο�  SSIZE
                        );

    DMA_CITER_ELINKNO(CHn)  = DMA_CITER_ELINKNO_CITER(count); //��ǰ��ѭ������
    DMA_BITER_ELINKNO(CHn)  = DMA_BITER_ELINKYES_BITER(count);//��ʼ��ѭ������

    DMA_CR &= ~DMA_CR_EMLM_MASK;                            // CR[EMLM] = 0

    DMA_NBYTES_MLNO(CHn) =   DMA_NBYTES_MLNO_NBYTES(BYTEs); // ͨ��ÿ�δ����ֽ�������������ΪBYTEs���ֽڡ�ע��ֵΪ0��ʾ����4GB */

    /* ���� DMA ���������Ĳ��� */
    DMA_SLAST(CHn)      =   -count;                              //����  Դ��ַ�ĸ���ֵ,��ѭ��������ָ�  Դ��ַ
    DMA_DLAST_SGA(CHn)  =   0;                                  //����Ŀ�ĵ�ַ�ĸ���ֵ,��ѭ��������ָ�Ŀ�ĵ�ַ���߱��ֵ�ַ
    DMA_CSR(CHn)        =   (0
                             | DMA_CSR_DREQ_MASK            //��ѭ��������ֹͣӲ������
                             | DMA_CSR_INTMAJOR_MASK        //��ѭ������������ж�
                            );

    /* ���� DMA ����Դ */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, CHn) = (0
            | DMAMUX_CHCFG_ENBL_MASK                        /* Enable routing of DMA request */
            | DMAMUX_CHCFG_SOURCE((ptxn >> 5) + DMA_Port_A) /* ͨ����������Դ:     */
                                             );

    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << (ptxn>>5));                                                               //����PORTx�˿�
    GPIO_PDDR_REG(GPIOx[(ptxn>>5)]) &= ~(1 << (ptxn & 0x1f));                                                       //���ö˿ڷ���Ϊ����
    PORT_PCR_REG(PORTX[(ptxn>>5)], (ptxn & 0x1F)) = ( 0
            | PORT_PCR_MUX(1)               // ����GPIO
            | PORT_PCR_IRQC(cfg & 0x03 )    // ȷ������ģʽ
            | ((cfg & 0xc0 ) >> 6)          // �����������������裬����û��
                                                    );
    GPIO_PDDR_REG(GPIOx[(ptxn>>5)]) &= ~(1 << (ptxn && 0x1F));                                                      //����ģʽ

    /* �����ж� */
    DMA_EN(CHn);                                    //ʹ��ͨ��CHn Ӳ������
    DMA_IRQ_EN(CHn);                                //����DMAͨ������
}


/*************************************************************************
*                             �Ĭ�Ƽ�DEMOK Kinetis����С��
*
*
*  �������ƣ�DMA_count_get
*  ����˵���������ۼӼ���ֵ
*  ����˵����DMA_CHn              ͨ���ţ�DMA_CH0 ~ DMA_CH15��
*  �������أ��ۼӼ���ֵ
*
*************************************************************************/
u32 DMA_count_get(DMA_CHn CHn)
{
    u32 temp =  count_init[CHn] - DMA_CITER_ELINKNO(CHn)  ;
    return temp;
}

void DMA_count_reset(DMA_CHn CHn)
{
    DMA_CITER_ELINKNO(CHn) = count_init[CHn] ;
}

/********************************************************************************
**Routine: myDMA_Config
**Description: DMA���ú���
               DMA_CHn����ָ����DMAͨ���ţ���Χ0~15��
               DMAMUX_Source����DMA����Դ����DMA.h�ļ�����ö�ٶ���
               S_Addr����DMA����Դ��ַ
               D_Addr����DMA����Ŀ�ĵ�ַ
               Block_Size����һ��DMA��������ݿ��С��/byte��
**Notes: Ĭ������£��̶����ȼ�ģʽ��ÿ����ͨ����������ȼ����ڸ�ͨ����ͨ���ţ�
        ����ͨ����С�����ȼ��͡���������Ĭ�����ȼ����ã�
********************************************************************************/
void myDMA_Config(uint8 DMA_CHn, uint8 DMAMUX_Source, uint32 S_Addr, uint32 D_Addr, uint16 Block_Size)
{
    /* the corresponding SIM clock gate to be enabled befor the DMAMUX module 
     register being initialized */
    SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
    /* Config DMAMUX for channel n */
    DMAMUX_CHCFG_REG(DMAMUX_BASE_PTR, DMA_CHn) = (0
        | DMAMUX_CHCFG_ENBL_MASK /* ʹ��DMAͨ�� */
        //| DMAMUX_CHCFG_TRIG_MASK              /* �������Դ���ģʽ��ע��ֻ��0~3ͨ��֧�� */
        | DMAMUX_CHCFG_SOURCE(DMAMUX_Source) /* ָ��DMA����Դ */
    );

    /* enable the DMA clock gate in SIM */
    SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;            /* DMAʱ���ſ��ϵ�Ĭ���Ǵ򿪵ģ������ⲽ�ɼӿɲ��� */
    DMA_CR = 0;                                 /* Ĭ�����ã���Ҫ��DMA������֮ǰ���ô˼Ĵ��� */
                                                //DMA_DCHPRIn                                                 /* Ĭ�����ȼ����ã����ﲻ����� */
    DMA_BASE_PTR->TCD[DMA_CHn].SADDR = S_Addr;  /* ����DMAԴ��ַ */
    DMA_BASE_PTR->TCD[DMA_CHn].DADDR = D_Addr;  /* ����DMAĿ���ַ */
    DMA_BASE_PTR->TCD[DMA_CHn].NBYTES_MLNO = 1; /* ÿ��minor loop����1���ֽ� */
    DMA_BASE_PTR->TCD[DMA_CHn].ATTR = (0
        | DMA_ATTR_SMOD(0)  /* Source modulo feature disabled */
        | DMA_ATTR_SSIZE(0) /* Source size, 8λ���� */
        | DMA_ATTR_DMOD(0)  /* Destination modulo feature disabled */
        | DMA_ATTR_DSIZE(0) /* Destination size, 8λ���� */
    );
    DMA_BASE_PTR->TCD[DMA_CHn].SOFF = 0x0000;                                       /* ÿ�β�����Դ��ַ��Դ��ַ������ */
    DMA_BASE_PTR->TCD[DMA_CHn].DOFF = 0x0001;                                       /* ÿ�β�����Ŀ���ַ��Ŀ���ַ����1  */
    DMA_BASE_PTR->TCD[DMA_CHn].SLAST = 0x00;                                        /* DMA���һ�����֮��major_loop˥����֮�󲻸���Դ��ַ */
    DMA_BASE_PTR->TCD[DMA_CHn].DLAST_SGA = -Block_Size;                             /* DMA���һ�����֮��major_loop˥����֮�����Ŀ���ַ,�ص���ʼ�� */
    DMA_BASE_PTR->TCD[DMA_CHn].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(Block_Size); /* 1��major loop, ��һ�δ�����=major_loop*minor_loop�����Ϊ2^15=32767 */
    DMA_BASE_PTR->TCD[DMA_CHn].BITER_ELINKNO = DMA_CITER_ELINKNO_CITER(Block_Size); /* BITERӦ�õ���CITER */

    DMA_BASE_PTR->TCD[DMA_CHn].CSR = 0;                      /* ������CSR��֮�������� */
    DMA_INT |= (1 << DMA_CHn);                               /* ����DMA��Ӧͨ���Ĵ�������жϣ���TCD_CSR_INTMAJOR����TCD_CSR_INTHALF���� */
    DMA_BASE_PTR->TCD[DMA_CHn].CSR |= DMA_CSR_INTMAJOR_MASK; /* ����DMA major_loop����ж� */
    DMA_BASE_PTR->TCD[DMA_CHn].CSR &= ~DMA_CSR_DREQ_MASK;    /* major_loop�ݼ�Ϊ0ʱ��Ҫ�Զ��ر�DMA����һֱ����DMA���� */

    /* DMA_ERQ�Ĵ�������Ҫ����λ��Ӧ��λ������DMA���� */
    DMA_ERQ &= ~(1 << DMA_CHn); /* �ر���Ӧͨ����DMA���������ý׶��ȹرգ��ٵ���myDMA_Start��������DMA */
}
/********************************************************************************
**Routine: myDMA_Start
**Description: ����DMA����ʹ��DMA����
**Notes: ��
********************************************************************************/
void myDMA_Start(uint8 DMA_CHn)
{
    DMA_ERQ |= (1 << DMA_CHn); /* ������Ӧͨ����DMA */
}
/********************************************************************************
**Routine: myDMA_Close
**Description: �ر���Ӧͨ����DMA����ֹͣDMA����
**Notes: 
********************************************************************************/
void myDMA_Close(uint8 DMA_CHn)
{
    DMA_ERQ &= ~(1 << DMA_CHn); /* ֹͣ��Ӧͨ����DMA */
}


