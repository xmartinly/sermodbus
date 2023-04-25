#include "adc.h"

#include "usart3.h"
float Adc_Data[3];
unsigned int AD_Value[3];  //用于存放ADC采样结果

u8 Timer_3_index, Timer_3_OK_Flg;

// extern int Timer_3_index,Timer_2_index,Timer_1_index;
// extern unsigned char Timer_3_OK_Flg,Dwin_Send_Curve_Flg;
// extern unsigned char Timer_ADS1115_OK_Flg;
int Sensor_index;
#define cy_max 20  //最大采样数，最小15
// extern int Sen_NH3[cy_max],Sen_H2S[cy_max],Sen_VOC[cy_max],Sen_4ETO[cy_max];
// extern int Sen_BDT1[cy_max],Sen_BDT2[cy_max],Sen_BDT3[cy_max],Sen_BDT4[cy_max];
// extern int ADS1115_AD_Value[16];
// extern unsigned int BAT_AD,PW_AD,BAT_AD_avg,PW_AD_avg;
// extern void SensorDataDispose(int* DisposeData,int count);
// extern int PUMP_AD;
// extern int BAT_Value,PW_Value;
// extern int QX_FS_AD,QX_FX_AD;
//初始化ADC
void Adc_Init(void) {
    Adc_GPIO_Init();
    Adc_Config();
    Adc_DMA_Iint();
    Adc_TIM3_Init();
}

//配置IO口
void Adc_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
}

//配置DMA1
void Adc_DMA_Iint(void) {
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;               // DMA外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;                   // DMA内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                       //内存作为传输的目的地址
    DMA_InitStructure.DMA_BufferSize = 6;                                    // DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;         //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                  //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //数据宽度32位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;          //数据宽度32位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                          //循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                      // DMA通道1具有高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                             //没有设置为内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel1, ENABLE);
}
//配置ADC1
void Adc_Config(void) {
    ADC_InitTypeDef ADC_InitStructure;

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                   // ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         //模数转换工作扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  //模数转换工作在单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //不使用外部触发转换
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               // ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 3;                              //顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);  // ADC1,ADC通道,采样时间为55.5周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_55Cycles5);

    ADC_DMACmd(ADC1, ENABLE);  //开启ADC的DMA支持

    ADC_Cmd(ADC1, ENABLE);  //使能指定的ADC1

    ADC_ResetCalibration(ADC1);  //使能复位校准
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;                        //等待复位校准结束
    ADC_StartCalibration(ADC1);  //开启AD校准
    while (ADC_GetCalibrationStatus(ADC1))
        ;  //等待校准结束

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

//配置定时器2
void Adc_TIM3_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // 4999,7199
    TIM_TimeBaseInitStruct.TIM_Period = 999;  // 10000;                   //100ms触发一次
    TIM_TimeBaseInitStruct.TIM_Prescaler = 7199;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

//获得ADC平均值
void TIM3_IRQHandler(void) {
    unsigned char i;
    unsigned int CY1, CY2, CY3;
    CY1 = 0;
    CY2 = 0;
    CY3 = 0;

    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
        Timer_3_index++;
        if (Timer_3_index >= 10)  // 1秒
        {
            Timer_3_index = 0;

            for (i = 0; i < 10; i++) {
                CY3 += AD_Value[0];
                CY2 += AD_Value[1];  //
                CY1 += AD_Value[2];  //

                ADC_SoftwareStartConvCmd(ADC1, ENABLE);
            }

            Adc_Data[0] = ((CY1 / 10.0) * (3300.0 / 4096)) / 0.265;
            Adc_Data[1] = ((CY2 / 10.0) * (3300.0 / 4096)) / 0.265;
            Adc_Data[2] = ((CY3 / 10.0) * (3300.0 / 4096)) / 0.265;

            Timer_3_OK_Flg = 0xaa;
        }
    }
}
void ConvertData(void) {
    if (Timer_3_OK_Flg == 0xaa) {
        Timer_3_OK_Flg = 0x00;
//        Float_To_U32(Adc_Data[0], 8);   //复制数据给寄存器
//        Float_To_U32(Adc_Data[1], 10);  //复制数据给寄存器
//        Float_To_U32(Adc_Data[2], 12);  //复制数据给寄存器
    }
}
