#ifndef DRV_UART_H
#define DRV_UART_H

#include "1_Middleware/3_System/Timestamp/sys_timestamp.h"
#include "usart.h"
#include "stm32h7xx_hal.h"
#include <string.h>

#define UART_BUFFER_SIZE 512

typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    UART_Callback Callback_Function;

    // 新增：当前串口实际使用的 DMA 接收长度
    uint16_t Rx_Buffer_Length;

    // 双缓冲
    uint8_t Rx_Buffer_0[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[UART_BUFFER_SIZE];

    // 正在接收的缓冲区
    uint8_t *Rx_Buffer_Active;

    // 已接收完成的缓冲区
    uint8_t *Rx_Buffer_Ready;

    // 最近一次接收时间戳
    uint64_t Rx_Time_Stamp;
};

extern volatile bool init_finished;

extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;
extern Struct_UART_Manage_Object UART4_Manage_Object;
extern Struct_UART_Manage_Object UART5_Manage_Object;
extern Struct_UART_Manage_Object UART6_Manage_Object;
extern Struct_UART_Manage_Object UART7_Manage_Object;
extern Struct_UART_Manage_Object UART8_Manage_Object;
extern Struct_UART_Manage_Object UART9_Manage_Object;
extern Struct_UART_Manage_Object UART10_Manage_Object;

// 给第三个参数默认值，这样旧的 2 参数调用也不用全改
void UART_Init(UART_HandleTypeDef *huart,
               UART_Callback Callback_Function,
               uint16_t Rx_Buffer_Length = UART_BUFFER_SIZE);

void UART_Reinit(UART_HandleTypeDef *huart);

uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

#endif