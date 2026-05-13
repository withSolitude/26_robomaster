//
// Created by Lenovo on 2025/12/10.
//
#include "dvc_Vofa.h"
#include <cstring>
Class_Vofa_JustFloat::Class_Vofa_JustFloat()
    : UART_Manage_Object(nullptr),
      Channel_Number(0),
      Send_Period_ms(20),
      Counter_ms(0)
{
    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        Channel_Pointer[i] = nullptr;
    }
}

/**
 *初始化
 *
 * @param send_func     发送函数指针(用这个发数据)
 * @param channel_num   通道数量
 */
void Class_Vofa_JustFloat::Init(UART_HandleTypeDef *huart,uint8_t channel_num,uint16_t send_period_ms)
{
    //UART_Init(huart,nullptr,UART_BUFFER_SIZE);

    // 选择对应的UART管理对象
    if (huart->Instance==USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance==USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance==USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance==UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance==USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }


    if (channel_num > MAX_CHANNEL)
        channel_num = MAX_CHANNEL;
    else if (channel_num == 0)
        channel_num = 1;

    Channel_Number = channel_num;
    Send_Period_ms = send_period_ms;
    Counter_ms = 0;

    // 清空
    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        Channel_Pointer[i] = nullptr;
    }
}

/**
 *@brief 绑定通道到一个float变量的地址
 *
 * @param index 通道索引
 * @param p_data 要发送的浮点数变量的地址
 */
void Class_Vofa_JustFloat::Bind_Channel(uint8_t index,float *p_data)
{
    if (index >= MAX_CHANNEL)
        return;

    Channel_Pointer[index] = p_data;
}

void Class_Vofa_JustFloat::TIM_1ms_PeriodElapsedCallback()
{
    if (UART_Manage_Object == nullptr || Channel_Number == 0)
        return;

    if (++Counter_ms >= Send_Period_ms)
    {
        Counter_ms = 0;
        Output();
    }
}

void Class_Vofa_JustFloat::Output(void)
{
    if (UART_Manage_Object == nullptr || Channel_Number == 0)
        return;

    // 一帧的长度
    const uint16_t frame_len = Channel_Number * sizeof(float) + 4;

    //
    uint8_t *tx_buffer = UART_Manage_Object->Tx_Buffer;
    uint8_t *p = tx_buffer;

    for (uint8_t i = 0; i < Channel_Number; i++)
    {
        float value = 0.0f;
        if (Channel_Pointer[i] != nullptr)
        {
            value = *Channel_Pointer[i];
        }

        std::memcpy(p,&value,sizeof(float));
        p += sizeof(float);
    }

    //JustFloat 帧尾:0x00 0x00 0x80 0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    uint16_t send_len = static_cast<uint16_t>(p-tx_buffer);
    // 发送数据
    UART_Send_Data(UART_Manage_Object->UART_Handler,tx_buffer,send_len);
}



