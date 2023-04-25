#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "adc.h"
#include "led.h"
#include "stm32f10x.h"
#include "task.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
float Conc[4], AdcValue[3], Triggers[4];
extern u16 SendCmd;

//开始检测//清零功能//休眠功能//暂停测量//清除报警
char* Control_Cmd[] = {"*START\r", "*ZERO\r", "*SLEEP\r", "*STANDby\r", "*CLS\r"};


void vTask1(void* pvParameters) {
    int index = 0;
    LED_G = 0;
    while (1) {
        Modbus_Service();
        if (index == 5) {
            LED_G ^= 1;
            index = 0;
        }
        //如果网络上没有发指令时，循环读取
        switch (index) {
            case 0:
                printf("*READ 1?\r");
                break;
            case 1:
                printf("*READ 2?\r");
                break;
            case 2:
                printf("*READ 3?\r");
                break;
            case 3:
                printf("*READ 4?\r");
                break;
            case 4:
                printf("*STATus?\r");
                break;
        }
        index++;
        vTaskDelay(100 / portTICK_RATE_MS);
        Uart2__Service(index);
        
    }
}
void vTask2(void* pvParameters) {
   
        vTaskDelay(250 / portTICK_RATE_MS);
        Modbus_Service();
//        ConvertData();
    
}
void vTask3(void* pvParameters) {
//    while (1) {
//        //判断是否接受到新指令，当不为0的时候就有新的指令到了
//        if (SendCmd != 0) {
//            printf("%s", Control_Cmd[SendCmd - 1]);
//        }
//        vTaskDelay(500 / portTICK_RATE_MS);
//        //判断是否发送成功
//        if (find_string("OK")) {
//            SendCmd = 0;
//            LED_R = 0;
//        } else {
//            LED_R = 1;
//        }
//    }
//    int index = 10;
//    while (1) {
//        if (index == 14) {
//            index = 10;
//        }
//        switch (index) {
//            case 10:
//                printf("*GAS:1:TRIgger?\r");
//                break;
//            case 11:
//                printf("*GAS:2:TRIgger?\r");
//                break;
//            case 12:
//                printf("*GAS:3:TRIgger?\r");
//                break;
//            case 13:
//                printf("*GAS:4:TRIgger?\r");
//                break;
//        }
//        index++;
//        vTaskDelay(100 / portTICK_RATE_MS);
//        Uart2__Service(index);
//    }
}


int main(void) {
    LED_Init();
    uart1_init(115200);  //-------------------调试-------------------
    uart2_init(19200);   //-------------------RS232-----------------
    uart3_init(115200);  //-------------------网络-------------------
    //Adc_Init();
    xTaskCreate(vTask1, "vTask1", 128, NULL, 1, NULL);
    //xTaskCreate(vTask2, "vTask2", 64, NULL, 2, NULL);
    //xTaskCreate(vTask3, "vTask3", 1000, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1)
        ;
}

void USART1_IRQHandler(void) {
    u8 Res;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        Res = USART_ReceiveData(USART1);  //(USART1->DR);
        printf("%c", Res);
    }
}
