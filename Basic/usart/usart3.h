#ifndef _USART3_H_
#define _USART3_H_
#include <stdio.h>

#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"

#define USART_REC_LEN 200  //定义最大接收字节数 200
#define Modbus_Addr 0x01

void uart3_init(u32 bound);
unsigned short getCRC16(volatile char* ptr, unsigned char len);
void Usart1_R_BUF_DataProc(void);
void Usart3_R_BUF_DataProc(void);
void USART3_IRQHandler(void);  //串口3中断服务程序
void Modbus_Service(void);
void Modbus_RegMap(void);
void RJ45_SendData(u8* buff, u8 len);
void Timer7_Init(void);
void TIM7_IRQHandler(void);
void Modbus_02_Solve(void);
void Modbus_01_Solve(void);
void Modbus_05_Solve(void);
void Modbus_15_Solve(void);
void Modbus_03_Solve(void);
void Modbus_06_Solve(void);
void Modbus_16_Solve(void);

void Float_To_U32(float value, u16 Holdaddr);
void Modbus_Function_3(void);
void Modbus_Function_6(void);

#endif
