#ifndef _USART2_H_
#define _USART2_H_
#include <stdio.h>

#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#define USART2_REC_LEN 100  //定义最大接收字节数 200

void uart2_init(u32 bound);  //
void TIM4_IRQHandler(void);  //
void TIM4_Set(char sta);
void TIM4_Init(u16 arr, u16 psc);
void USART2_IRQHandler(void);  //串口2中断服务程序
unsigned char find_string(char* p);
void clear_USART2_RX_buf(void);
void Uart2__Service(int index);

#endif
