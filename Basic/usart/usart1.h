#ifndef _USART_H_
#define _USART_H_
#include <stdio.h>

#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

void uart1_init(u32 bound);
int fputc(int ch, FILE* f);
void USART1_Putc(unsigned char c);
void USART1_Puts(char* str);
void USART1_IRQHandler(void);
#endif
