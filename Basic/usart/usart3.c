#include "usart3.h"

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误
u8 USART3_RX_BUF[USART_REC_LEN];  //接收缓冲,最大USART_REC_LEN个字节.
u8 USART3_TX_BUF[USART_REC_LEN];  //接收缓冲,最大USART_REC_LEN个字节.
int USART3_TxCounter, USART3_RxCounter;
u16 RJ45_Frame_Distance = 10;  //数据帧最小间隔（ms),超过此时间则认为是下一帧

u8 uart3_r_Flg;
u8 USART_FrameFlag;

// Modbus寄存器和单片机寄存器的映射关系
vu32* Modbus_InputIO[100];   //输入开关量寄存器指针(这里使用的是位带操作)
vu32* Modbus_OutputIO[100];  //输出开关量寄存器指针(这里使用的是位带操作)
u16* Modbus_HoldReg[100];    //保持寄存器指针
//u16 Modbus_Float[16];
u16 Modbus_Float[14];
u16 SendCmd;
extern u16 Run_Status;

/*浮点数与IEEE754格式转换*/
typedef union {
    float ul_Temp;
    u8 uc_Buf[4];   //小数转换成IEEE754格式的数组
    u16 us_Buf[2];  //用于modbus协议小数转换的数组
} un_DtformConver;

u16 USART_RX_STA = 0;  //接收状态标记

/////
/*
函数功能：将小数转换成IEEE754数据类型，用于modbus协议传输数据
函数参数：value 小数值    Holdaddr 从机保持寄存器地址
*/
//方式1.0
void Float_To_U32(float value, u16 Holdaddr) {
    un_DtformConver Data;
    Data.ul_Temp = value;                         //获取小数值
    Modbus_Float[Holdaddr] = Data.us_Buf[0];      //获取低16位
    Modbus_Float[Holdaddr + 1] = Data.us_Buf[1];  //获取高16位
}

void uart3_init(u32 bound) {
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能UART3所在GPIOB的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    //串口使用的GPIO口配置
    // Configure USART3 Rx (PB.11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Configure USART3 Tx (PB.10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                            //根据指定的参数初始化VIC寄存器
    // USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                      //串口波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                      //字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                           //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                              //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                  //收发模式
    USART_Init(USART3, &USART_InitStructure);                                        //初始化串口1
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                   //开启串口接受中断
    USART_Cmd(USART3, ENABLE);                                                       //使能串口1
    // GPIO_InitTypeDef  GPIO_InitStructure;
    //   -------------方向控制IO----------
    //	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);	        //使能GPIOB,GPIOC端口时钟
    //	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
    //	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //	GPIO_Init(GPIOE, &GPIO_InitStructure);
    //  GPIO_ResetBits(GPIOE,   GPIO_Pin_15 );//输出1
    Timer7_Init();    //定时器7初始化，用于监视空闲时间
    Modbus_RegMap();  // Modbus寄存器映射
}

unsigned short getCRC16(volatile char* ptr, unsigned char len) {
    unsigned char i;
    unsigned short crc = 0xFFFF;
    if (len == 0) {
        len = 1;
    }
    while (len--) {
        crc ^= *ptr;
        for (i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return (crc);
}

// void Usart3_R_BUF_DataProc(void)
//{
//		unsigned short crcresult;
//		u8 temp[2];
//
//	if(uart3_r_Flg==0xaa)
//	{
//		uart3_r_Flg=0x00;
//		crcresult = getCRC16(USART3_RX_BUF,USART3_RxCounter-2);
//		temp[1] = crcresult & 0xff;
//     temp[0] = (crcresult >> 8) & 0xff;
//		if((USART3_RX_BUF[USART3_RxCounter-1] == temp[0])&&(USART3_RX_BUF[USART3_RxCounter-2] == temp[1]))//crc校验错误不应答
//		{
//			if(USART3_RX_BUF[0]==Modbus_Addr)
//			{

//		//	Usart_LED=1;
//				USART3_RxCounter=0;

//				if(USART3_RX_BUF[1]==0x03)
//				{
//					Modbus_Function_3();
//				}
//				else if(USART3_RX_BUF[1]==0x06)
//				{
//					Modbus_Function_6();
//				}
//			//	Usart_LED=0;

//			}
//		}

//		else
//		{
//			USART3_RX_BUF[0]=0;
//			USART3_RxCounter=0;//CRC校验不通过就清0
//		}
//		USART3_RxCounter=0;
//		USART3_RX_BUF[0]=0;
//	}
//}

//定时器7初始化
void Timer7_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // TIM7时钟使能
    // TIM7初始化设置
    TIM_TimeBaseStructure.TIM_Period = RJ45_Frame_Distance * 10;  //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 7200;                   //设置用来作为TIMx时钟频率除数的预分频值 设置计数频率为10kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   // TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);               //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                    // TIM7 允许更新中断
    // TIM7中断分组配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            // TIM7中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;         //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            // IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);                            //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
///////////////////////////////////////////////////////////////////////////////////////
//用定时器7判断接收空闲时间，当空闲时间大于指定时间，认为一帧结束
//定时器7中断服务程序
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除中断标志
        TIM_Cmd(TIM2, DISABLE);                      //停止定时器
        USART_FrameFlag = 1;                         //置位帧结束标记
    }
}
//-------------------------------------------------------------------------------------------------------------------
//
//                                             串口3中断服务程序
//
// 																									DTU对外输出
//
//-------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void) {  //串口3中断服务程序
    u8 res;
    u8 err;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        if (USART_GetFlagStatus(USART3, USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)) {
            err = 1;  //检测到噪音、帧错误或校验错误
        } else {
            err = 0;
        }
        res = USART_ReceiveData(USART3);  //读接收到的字节，同时相关标志自动清除
        if ((USART3_RxCounter < 2047) && (err == 0)) {
            USART3_RX_BUF[USART3_RxCounter] = res;
            USART3_RxCounter++;
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除定时器溢出中断
            TIM_SetCounter(TIM2, 0);                     //当接收到一个新的字节，将定时器7复位为0，重新计时（相当于喂狗）
            TIM_Cmd(TIM2, ENABLE);                       //开始计时
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
//
//                                             Modbus地址映射表
//
//                                              对外输出
//
//-------------------------------------------------------------------------------------------------------------------
void Modbus_RegMap(void) {
    int bb;
    bb = 0;
    Modbus_HoldReg[bb++] = &Modbus_Float[0];  //数据1
    Modbus_HoldReg[bb++] = &Modbus_Float[1];
    Modbus_HoldReg[bb++] = &Modbus_Float[2];  //数据2
    Modbus_HoldReg[bb++] = &Modbus_Float[3];
    Modbus_HoldReg[bb++] = &Modbus_Float[4];  //数据3
    Modbus_HoldReg[bb++] = &Modbus_Float[5];
    Modbus_HoldReg[bb++] = &Modbus_Float[6];  //数据4
    Modbus_HoldReg[bb++] = &Modbus_Float[7];
    Modbus_HoldReg[bb++] = &Modbus_Float[8];    // ADC1
    Modbus_HoldReg[bb++] = &Modbus_Float[9];
    Modbus_HoldReg[bb++] = &Modbus_Float[10];   // ADC2
    Modbus_HoldReg[bb++] = &Modbus_Float[11];
    Modbus_HoldReg[bb++] = &Modbus_Float[12];   // ADC3
    Modbus_HoldReg[bb++] = &Modbus_Float[13];
    Modbus_HoldReg[bb++] = &Run_Status;         //运行模式
    //保持寄存器指针指向
    //        Modbus_HoldReg[0]=(u16*)&testData1;//测试数据1
    //        Modbus_HoldReg[1]=((u16*)&testData1)+1;//测试数据1
    //        Modbus_HoldReg[2]=(u16*)&testData2;//测试数据2
    //        Modbus_HoldReg[3]=((u16*)&testData2)+1;//测试数据2
    //		Modbus_HoldReg[bb++]=(u16*)&reset_Flg;//重启标志位
}
//////////////////////////////////////////////////////////////////////////////
//发送n个字节数据
// buff:发送区首地址
// len：发送的字节数
void RJ45_SendData(u8* buff, u8 len) {
    while (len--) {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
            ;  //等待发送区为空
        USART_SendData(USART3, *(buff++));
    }
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
        ;  //等待发送完成
}

///////////////////////////////////////////////////////////////////////////////////////
// RS485服务程序，用于处理接收到的数据(请在主函数中循环调用)
u16 startRegAddr;
u16 RegNum;
u16 calCRC;
void Modbus_Service(void) {
    u16 recCRC;
    if (USART_FrameFlag == 1) {
        if (USART3_RX_BUF[0] == Modbus_Addr) {                                                                                                                                                                       //地址正确
            if ((USART3_RX_BUF[1] == 01) || (USART3_RX_BUF[1] == 02) || (USART3_RX_BUF[1] == 03) || (USART3_RX_BUF[1] == 05) || (USART3_RX_BUF[1] == 06) || (USART3_RX_BUF[1] == 15) || (USART3_RX_BUF[1] == 16)) {  //功能码正确
                startRegAddr = (((u16)USART3_RX_BUF[2]) << 8) | USART3_RX_BUF[3];                                                                                                                                    //获取寄存器起始地址
                if (USART3_RX_BUF[1] == 0x03 && USART3_RX_BUF[2] == 0x40) {
                    startRegAddr = USART3_RX_BUF[3];  //获取寄存器起始地址
                }
                if (USART3_RX_BUF[1] == 0x06 && USART3_RX_BUF[2] == 0x10) {
                    startRegAddr = USART3_RX_BUF[3];  //获取寄存器起始地址
                }
                if (startRegAddr < 100) {                                                                              //寄存器地址在范围内
                    calCRC = getCRC16((char*)USART3_RX_BUF, USART3_RxCounter - 2);                                     //计算所接收数据的CRC
                    recCRC = USART3_RX_BUF[USART3_RxCounter - 2] | (((u16)USART3_RX_BUF[USART3_RxCounter - 1]) << 8);  //接收到的CRC(低字节在前，高字节在后)
                    if (calCRC == recCRC) {                                                                            //CRC校验正确
                        USART3_RxCounter = 0;
                        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                        switch (USART3_RX_BUF[1]) {  //根据不同的功能码进行处理
                            case 1:   //读输出开关量
                                Modbus_01_Solve();
                                break;
                            case 2:                 //读输入开关量
                                Modbus_02_Solve();
                                break;
                            case 3:   //读多个寄存器
                                Modbus_03_Solve();
                                break;
                            case 5:   //写单个输出开关量
                                Modbus_05_Solve();
                                break;
                            case 15:   //写多个输出开关量
                                Modbus_15_Solve();
                                break;
                            case 6:   //写单个寄存器
                                Modbus_Function_6();
                                break;
                            case 16:   //写多个寄存器
                                Modbus_16_Solve();
                                break;
                        }
                        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    } else {  // CRC校验错误
                        USART3_RxCounter = 0;
                        USART3_TX_BUF[0] = USART3_RX_BUF[0];
                        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
                        USART3_TX_BUF[2] = 0x04;  //异常码
                        RJ45_SendData(USART3_TX_BUF, 3);
                    }
                } else {  //寄存器地址超出范围
                    USART3_RxCounter = 0;
                    USART3_TX_BUF[0] = USART3_RX_BUF[0];
                    USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
                    USART3_TX_BUF[2] = 0x02;  //异常码
                    RJ45_SendData(USART3_TX_BUF, 3);
                }
            } else {  //功能码错误
                USART3_RxCounter = 0;
                USART3_TX_BUF[0] = USART3_RX_BUF[0];
                USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
                USART3_TX_BUF[2] = 0x01;  //异常码
                RJ45_SendData(USART3_TX_BUF, 3);
            }
        }
        USART_FrameFlag = 0;   //复位帧结束标志
        USART3_TxCounter = 0;  //接收计数器清零
    }
}

// Modbus功能码02处理程序
//读输入开关量
void Modbus_02_Solve(void) {
    u16 ByteNum;
    u16 i;
    RegNum = (((u16)USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];  //获取寄存器数量
    if ((startRegAddr + RegNum) < 100) {                         //寄存器地址+数量在范围内
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        ByteNum = RegNum / 8;  //字节数
        if (RegNum % 8) {
            ByteNum += 1;  //如果位数还有余数，则字节数+1
        }
        USART3_TX_BUF[2] = ByteNum;  //返回要读取的字节数
        for (i = 0; i < RegNum; i++) {
            if (i % 8 == 0) {
                USART3_TX_BUF[3 + i / 8] = 0x00;
            }
            USART3_TX_BUF[3 + i / 8] >>= 1;  //低位先发送
            USART3_TX_BUF[3 + i / 8] |= ((*Modbus_InputIO[startRegAddr + i]) << 7) & 0x80;
            if (i == RegNum - 1) {  //发送到最后一个位了
                if (RegNum % 8) {
                    USART3_TX_BUF[3 + i / 8] >>= 8 - (RegNum % 8);  //如果最后一个字节还有余数，则剩余MSB填充0
                }
            }
        }
        calCRC = getCRC16((char*)USART3_TX_BUF, ByteNum + 3);
        USART3_TX_BUF[ByteNum + 3] = calCRC & 0xFF;
        USART3_TX_BUF[ByteNum + 4] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, ByteNum + 5);
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

// Modbus功能码01处理程序
//读输出开关量
void Modbus_01_Solve(void) {
    u16 ByteNum;
    u16 i;
    RegNum = (((u16)USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];  //获取寄存器数量
    if ((startRegAddr + RegNum) < 100) {                         //寄存器地址+数量在范围内
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        ByteNum = RegNum / 8;  //字节数
        if (RegNum % 8) {
            ByteNum += 1;  //如果位数还有余数，则字节数+1
        }
        USART3_TX_BUF[2] = ByteNum;  //返回要读取的字节数
        for (i = 0; i < RegNum; i++) {
            if (i % 8 == 0) {
                USART3_TX_BUF[3 + i / 8] = 0x00;
            }
            USART3_TX_BUF[3 + i / 8] >>= 1;  //低位先发送
            USART3_TX_BUF[3 + i / 8] |= ((*Modbus_OutputIO[startRegAddr + i]) << 7) & 0x80;
            if (i == RegNum - 1) {  //发送到最后一个位了
                if (RegNum % 8) {
                    USART3_TX_BUF[3 + i / 8] >>= 8 - (RegNum % 8);  //如果最后一个字节还有余数，则剩余MSB填充0
                }
            }
        }
        calCRC = getCRC16((char*)USART3_TX_BUF, ByteNum + 3);
        USART3_TX_BUF[ByteNum + 3] = calCRC & 0xFF;
        USART3_TX_BUF[ByteNum + 4] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, ByteNum + 5);
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

// Modbus功能码05处理程序
//写单个输出开关量
void Modbus_05_Solve(void) {
    if (startRegAddr < 100) {  //寄存器地址在范围内
        if ((USART3_RX_BUF[4] == 0xFF) || (USART3_RX_BUF[5] == 0xFF)) {
            *Modbus_OutputIO[startRegAddr] = 0x01;
        } else {
            *Modbus_OutputIO[startRegAddr] = 0x00;
        }
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        USART3_TX_BUF[2] = USART3_RX_BUF[2];
        USART3_TX_BUF[3] = USART3_RX_BUF[3];
        USART3_TX_BUF[4] = USART3_RX_BUF[4];
        USART3_TX_BUF[5] = USART3_RX_BUF[5];
        calCRC = getCRC16((char*)USART3_TX_BUF, 6);
        USART3_TX_BUF[6] = calCRC & 0xFF;
        USART3_TX_BUF[7] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, 8);
    } else {  //寄存器地址超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

// Modbus功能码15处理程序
//写多个输出开关量
void Modbus_15_Solve(void) {
    u16 i;
    RegNum = (((u16)USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];  //获取寄存器数量
    if ((startRegAddr + RegNum) < 100) {                         //寄存器地址+数量在范围内
        for (i = 0; i < RegNum; i++) {
            if (USART3_RX_BUF[7 + i / 8] & 0x01) {
                *Modbus_OutputIO[startRegAddr + i] = 0x01;
            } else {
                *Modbus_OutputIO[startRegAddr + i] = 0x00;
            }
            USART3_RX_BUF[7 + i / 8] >>= 1;  //从低位开始
        }
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        USART3_TX_BUF[2] = USART3_RX_BUF[2];
        USART3_TX_BUF[3] = USART3_RX_BUF[3];
        USART3_TX_BUF[4] = USART3_RX_BUF[4];
        USART3_TX_BUF[5] = USART3_RX_BUF[5];
        calCRC = getCRC16((char*)USART3_TX_BUF, 6);
        USART3_TX_BUF[6] = calCRC & 0xFF;
        USART3_TX_BUF[7] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, 8);
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

// Modbus功能码03处理程序
//读保持寄存器
void Modbus_03_Solve(void) {
    u8 i;
    RegNum = (((u16)USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];  //获取寄存器数量
    if ((startRegAddr + RegNum) < 1000) {                        //寄存器地址+数量在范围内
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        USART3_TX_BUF[2] = RegNum * 2;
        for (i = 0; i < RegNum; i++) {
            USART3_TX_BUF[3 + i * 2] = *Modbus_HoldReg[startRegAddr + i] & 0xFF;         //先发送低字节
            USART3_TX_BUF[4 + i * 2] = (*Modbus_HoldReg[startRegAddr + i] >> 8) & 0xFF;  //后发送高字节
        }
        calCRC = getCRC16((char*)USART3_TX_BUF, RegNum * 2 + 3);
        USART3_TX_BUF[RegNum * 2 + 3] = calCRC & 0xFF;
        USART3_TX_BUF[RegNum * 2 + 4] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, RegNum * 2 + 5);
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

// Modbus功能码06处理程序
//写单个保持寄存器
void Modbus_06_Solve(void) {
    *Modbus_HoldReg[startRegAddr] = USART3_RX_BUF[4];               //低字节在前
    *Modbus_HoldReg[startRegAddr] |= ((u16)USART3_RX_BUF[5]) << 8;  //高字节在后
    USART3_TX_BUF[0] = USART3_RX_BUF[0];
    USART3_TX_BUF[1] = USART3_RX_BUF[1];
    USART3_TX_BUF[2] = USART3_RX_BUF[2];
    USART3_TX_BUF[3] = USART3_RX_BUF[3];
    USART3_TX_BUF[4] = USART3_RX_BUF[4];
    USART3_TX_BUF[5] = USART3_RX_BUF[5];
    calCRC = getCRC16((char*)USART3_TX_BUF, 6);
    USART3_TX_BUF[6] = calCRC & 0xFF;
    USART3_TX_BUF[7] = (calCRC >> 8) & 0xFF;
    RJ45_SendData(USART3_TX_BUF, 8);
}

// Modbus功能码16处理程序
//写多个保持寄存器
void Modbus_16_Solve(void) {
    u8 i;
    RegNum = (((u16)USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];  //获取寄存器数量
    if ((startRegAddr + RegNum) < 1000) {                        //寄存器地址+数量在范围内
        for (i = 0; i < RegNum; i++) {
            *Modbus_HoldReg[startRegAddr + i] = USART3_RX_BUF[7 + i * 2];               //低字节在前
            *Modbus_HoldReg[startRegAddr + i] |= ((u16)USART3_RX_BUF[8 + i * 2]) << 8;  //高字节在后
        }
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1];
        USART3_TX_BUF[2] = USART3_RX_BUF[2];
        USART3_TX_BUF[3] = USART3_RX_BUF[3];
        USART3_TX_BUF[4] = USART3_RX_BUF[4];
        USART3_TX_BUF[5] = USART3_RX_BUF[5];
        calCRC = getCRC16((char*)USART3_TX_BUF, 6);
        USART3_TX_BUF[6] = calCRC & 0xFF;
        USART3_TX_BUF[7] = (calCRC >> 8) & 0xFF;
        RJ45_SendData(USART3_TX_BUF, 8);
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}

void Modbus_Function_6(void) {
    signed short int RegNum;
    int Addr;
    //	int para_addr;//
    u16 tempdress = 0;
    u16 crcresult;
    // para_addr=0;
    tempdress = (USART3_RX_BUF[2] << 8) + USART3_RX_BUF[3];  //获取寄存器起始地址
    RegNum = ((USART3_RX_BUF[4]) << 8) | USART3_RX_BUF[5];   //获取寄存器数值
    if ((tempdress > 0x1000) & (tempdress <= 0x1005)) {      //地址范围
        Addr = tempdress & 0x0f;
        if (RegNum) {
            SendCmd = Addr;
            USART3_TX_BUF[0] = Modbus_Addr;
            USART3_TX_BUF[1] = USART3_RX_BUF[1];
            USART3_TX_BUF[2] = USART3_RX_BUF[2];
            USART3_TX_BUF[3] = USART3_RX_BUF[3];
            USART3_TX_BUF[4] = USART3_RX_BUF[4];
            USART3_TX_BUF[5] = USART3_RX_BUF[5];
            USART3_TxCounter = 6;
            crcresult = getCRC16((char*)USART3_TX_BUF, USART3_TxCounter);
            USART3_TX_BUF[USART3_TxCounter] = crcresult & 0xff;
            USART3_TX_BUF[USART3_TxCounter + 1] = (crcresult >> 8) & 0xff;
            RJ45_SendData(USART3_TX_BUF, USART3_TxCounter + 2);
        } else {  //寄存器地址+数量超出范围
            USART3_TX_BUF[0] = USART3_RX_BUF[0];
            USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
            USART3_TX_BUF[2] = 0x02;  //异常码
            RJ45_SendData(USART3_TX_BUF, 3);
        }
    } else {  //寄存器地址+数量超出范围
        USART3_TX_BUF[0] = USART3_RX_BUF[0];
        USART3_TX_BUF[1] = USART3_RX_BUF[1] | 0x80;
        USART3_TX_BUF[2] = 0x02;  //异常码
        RJ45_SendData(USART3_TX_BUF, 3);
    }
}
