#ifndef __ADC_H
#define __ADC_H
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_DMA.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

extern char reset_Flg;
void Adc_Init(void);
void Adc_GPIO_Init(void);
void Adc_DMA_Iint(void);
void Adc_Config(void);
void Adc_TIM3_Init(void);
void ADS1115_Caiji(void);
void ConvertData(void);
#endif
