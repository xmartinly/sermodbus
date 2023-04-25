#include "usart2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "string.h"
#include "usart3.h"
u8 USART2_RX_BUF[USART2_REC_LEN];  //接收缓冲,最大USART_REC_LEN个字节.
u8 USART2_TX_BUF[USART2_REC_LEN];  //接收缓冲,最大USART_REC_LEN个字节.
u8 RxCounter2;
u8 uart2_r_Flg;
u16 Run_Status;


/*************************************************************************
 * 函数名称: uart2_init(u32 bound)
 * 描    述: 串口2初始化函数
 * 输    入: bound——波特率
 * 输    出: 无
 * 返    回: 无
 * 说    明: 注意串口设置
 *************************************************************************/
void uart2_init(u32 bound) {  //---------------------------------COM_2 INIT ---------------------------------
    /* Private variables ---------------------------------------------------------*/
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    // GPIO端口设置
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable USARTz Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能USART1，GPIOA时钟
    // Usart1 NVIC 配置
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    /* Enable the USARTz Interrupt */
    // Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                            //根据指定的参数初始化VIC寄存器
    // USART1_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  // PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);           //初始化GPIOA.9
    // USART1_RX	  GPIOA.3初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;              // PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //初始化GPIOA.10
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* Configure USARTz */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable USARTz Receive and Transmit interrupts */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    // USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    /* Enable the USARTz */
    USART_Cmd(USART2, ENABLE);
    TIM4_Init(99, 7199);  // 10ms中断
    TIM4_Set(0);          //关闭定时器4
}

//
/*************************************************************************
* 函数名称: TIM4_IRQHandler(void)

* 描    述: 定时器4中断服务程序
* 输    入:
* 输    出: 无
* 返    回: 无
* 说    明: 注意串口设置
*************************************************************************/
void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {  //是更新中断
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);       //清除TIMx更新中断标志
        {
            TIM4_Set(0);      //关闭TIM4
            uart2_r_Flg = 1;  //置位帧结束标记
        }
    }
}
//设置TIM4的开关
// sta:0，关闭;1,开启;
/*************************************************************************
* 函数名称: TIM4_Set(char sta)

* 描    述: 设置TIM4的开关
* 输    入: sta:0，关闭;1,开启;
* 输    出: 无
* 返    回: 无
* 说    明: 注意串口设置
*************************************************************************/
void TIM4_Set(char sta) {
    if (sta) {
        TIM_SetCounter(TIM4, 0);  //计数器清空
        TIM_Cmd(TIM4, ENABLE);    //使能TIMx
    } else {
        TIM_Cmd(TIM4, DISABLE);  //关闭定时器4
    }
}
//配置TIM4预装载周期值
void TIM4_SetARR(u16 period) {
    TIM_SetCounter(TIM4, 0);  //计数器清空
    TIM4->ARR &= 0x00;        //先清预装载周期值为0
    TIM4->ARR |= period;      //更新预装载周期值
}

/*************************************************************************
* 函数名称: void TIM4_Init(u16 arr,u16 psc)

* 描    述: 通用定时器中断初始化  //这里始终选择为APB1的2倍，而APB1为36M
* 输    入: arr：自动重装值   psc：时钟预分频数
* 输    出: 无
* 返    回: 无
* 说    明: 注意串口设置
*************************************************************************/
void TIM4_Init(u16 arr, u16 psc) {
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //时钟使能//TIM4时钟使能
    //定时器TIM3初始化
    TIM_TimeBaseStructure.TIM_Period = arr;                      //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                   //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              //根据指定的参数初始化TIMx的时间基数单位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);                   //使能指定的TIM4中断,允许更新中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                            //根据指定的参数初始化VIC寄存器
}
/*******************************************************************************
 * 函数名称: USART2_IRQHandler
 * 描    述: 串口中断函数
 * 输入参数: 无
 * 输    出: 无
 * 返    回: 无
 * 说    明: 接收来自EC20模块的数据
 *******************************************************************************/
void USART2_IRQHandler(void) {  //串口2中断服务程序
    u8 Res;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        /* Read one byte from the receive data register */
        Res = USART_ReceiveData(USART2);  //读取接收到的数据
        USART2_RX_BUF[RxCounter2] = Res;
        RxCounter2++;
        if (RxCounter2 >= USART2_REC_LEN) {
            USART2_RX_BUF[0] = 0;
            RxCounter2 = 0;
        }
        TIM_SetCounter(TIM4, 0);  //计数器清空
        TIM4_Set(1);              //使能定时器4的中断
        /* Read one byte from the receive data register */
    }
}
/*******************************************************************************
 * 函数名称: find_string
 * 描    述: 判断缓存中是否含有指定的字符串
 * 输入参数: 字符串 p
 * 输    出: 无
 * 返    回: unsigned char:1 找到指定字符，0 未找到指定字符
 * 说    明: 调用extern char *strstr(char *str1, char *str2)该函数，
 *******************************************************************************/
unsigned char find_string(char* p) {
    unsigned int rx_size, resp_len;
    rx_size = strlen((char*)USART2_RX_BUF);
    resp_len = strlen(p);
    if (rx_size > resp_len) {
        if (strstr((char*)USART2_RX_BUF, p) != 0) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}
/*************************************************************************
 * 函数名称: clear_EC20_buf
 * 描    述: 清除串口2缓存数据
 * 输    入: 无
 * 输    出: 无
 * 返    回: 无
 * 说    明: 无
 *************************************************************************/
void clear_USART2_RX_buf(void) {
    if (RxCounter2 > 0) {
        memset(USART2_RX_BUF, 0, USART2_REC_LEN);
        RxCounter2 = 0;
    }
}
/*************************************************************************
 * 函数名称: Uart2__Service
 * 描    述: 串口2数据处理函数
 * 输    入: 无
 * 输    出: 无
 * 返    回: 无
 * 说    明: 无
 *************************************************************************/
void Uart2__Service(int index) {
    double d_readVal = 0;
    if( uart2_r_Flg == 1 && index == 5) {
        /* P3000 STATUS
             * INIT
             * START
             * MEAS
             * CAL
             * ERROR
             * ADJUST
             * STANDBY
             * OVERRANGE
             *
             * E3000 STATUS
             *
             * INIT
             * ACCL
             * MEAS
             * CALEXT
             * CALINT
             * PROOF
             * ERROR
             * SLEEP
             * PURGE
             * STANDBY
             */
        Run_Status = 0;
        if (find_string("INIT")) {
            Run_Status = 1;
        } else if (find_string("START")) {
            Run_Status = 2;
        } else if (find_string("MEAS")) {
            Run_Status = 3;
        } else if (find_string("CAL")) {
            Run_Status = 4;
        } else if (find_string("ERROR")) {
            Run_Status = 5;
        } else if (find_string("ADJUST")) {
            Run_Status = 6;
        } else if (find_string("STANDBY")) {
            Run_Status = 7;
        } else if (find_string("OVERRANGE")) {
            Run_Status = 8;
        } else if (find_string("ACCL")) {
            Run_Status = 9;
        } else if (find_string("CALEXT")) {
            Run_Status = 10;
        } else if (find_string("CALINT")) {
            Run_Status = 11;
        } else if (find_string("PROOF")) {
            Run_Status = 12;
        } else if (find_string("PURGE")) {
            Run_Status = 13;
        } else if (find_string("SLEEP")) {
            Run_Status = 14;
        }
    }
    if(uart2_r_Flg == 1) {
        int i_idx = index;
        if (
            find_string("g/a") ||
            find_string("mbar*l/s") ||
            find_string("ppm") ||
            find_string("oz/yr") ||
            find_string("Torr*l/s") ||
            find_string("Pa*m3/s") ||
            find_string("atm*cc/s")
        ) {  //获取反馈
            d_readVal = strtod((const char*)USART2_RX_BUF, NULL);
            uart2_r_Flg = 0;
            RxCounter2 = 0;
            Float_To_U32(d_readVal, (i_idx - 1) * 2); //传递给modbus
        }
    }
    clear_USART2_RX_buf();
    uart2_r_Flg = 0;
    RxCounter2 = 0;
}
