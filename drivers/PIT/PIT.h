//////////////////////////////////////////
////         杭州红树伟业      ///////////
///   http://shop36538723.taobao.com /////
///                                  ////
///     感谢野火提供的库文件          ///
///           2015-4-10               ///
//////////////////////////////////////////

#ifndef _PIT_H_
#define _PIT_H_



typedef enum PITn
{
    PIT0,
    PIT1,
    PIT2,
    PIT3
} PITn;


void       pit_init(PITn, u32 cnt);                                               //初始化PITn，并设置定时时间(单位为bus时钟周期)
#define     pit_init_ms(PITn,ms)          pit_init(PITn,ms * bus_clk_khz);         //初始化PITn，并设置定时时间(单位为 ms)

#define     PIT_Flag_Clear(PITn)          PIT_TFLG(PITn)|=PIT_TFLG_TIF_MASK        //清中断标志




void        pit_dma_init(PITn pitn, u32 cnt);
#define     pit_dma_init_ms(PITn,ms)     ASSERT( ((u64)(ms * bus_clk_khz)>>32)==0   );\
                                         pit_dma_init(PITn,ms * bus_clk_khz)

#endif  //_PIT_H_
