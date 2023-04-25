/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION 1                          //为1时RTOS使用抢占式调度器，为0时RTOS使用协作式调度器（时间片）
#define configUSE_IDLE_HOOK 0                           //设置为1使用空闲钩子（Idle Hook类似于回调函数），0忽略空闲钩子。
#define configUSE_TICK_HOOK 0                           //设置为1使用时间片钩子（Tick Hook），0忽略时间片钩子。
#define configCPU_CLOCK_HZ ((unsigned long)72000000)    // CPU内核时钟频率
#define configTICK_RATE_HZ ((TickType_t)100)            // RTOS 系统节拍中断的频率。即一秒中断的次数，每次中断RTOS都会进行任务调度。
#define configMAX_PRIORITIES (5)                        //配置应用程序有效的优先级数目 0优先级最小
#define configMINIMAL_STACK_SIZE ((unsigned short)128)  //定义空闲任务使用的堆栈大小。
#define configTOTAL_HEAP_SIZE ((size_t)(10 * 1024))     // RTOS内核总计可用的有效的RAM大小 这边为17K
#define configMAX_TASK_NAME_LEN (16)                    //定义的任务名称长度最大值 包括\0
#define configUSE_TRACE_FACILITY 0                      //设置成1表示启动可视化跟踪调试，会激活一些附加的结构体成员和函数。
#define configUSE_16_BIT_TICKS 0                        //定义为1使用16位计数器 为0使用32位 对于任务最大延时或阻塞时间，16位计数器是262秒，而32位是17179869秒。
#define configIDLE_SHOULD_YIELD 1                       //当和空闲任务相同优先级的用户任务就绪时 空闲任务是否让出

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 0              //设置成1表示使用协程，0表示不使用协程。如果使用协程，必须在工程中包含croutine.c文件。
#define configMAX_CO_ROUTINE_PRIORITIES (2)  //应用程序协程（Co-routines）的有效优先级数目

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 0
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1

/* This is the raw value as per the Cortex-M3 NVIC.  Values can be 255
(lowest) to 0 (1?) (highest). */
#define configKERNEL_INTERRUPT_PRIORITY 255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 191 /* equivalent to 0xb0, or priority 11. */

/* This is the value being used as per the ST library which permits 16
priority values, 0 to 15.  This must correspond to the
configKERNEL_INTERRUPT_PRIORITY setting.  Here 15 corresponds to the lowest
NVIC value of 255. */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15

#endif /* FREERTOS_CONFIG_H */