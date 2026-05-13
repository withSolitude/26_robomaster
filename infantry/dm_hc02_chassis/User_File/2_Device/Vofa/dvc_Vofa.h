/**
 * @file dvc_Vofa.h
 * @brief VOFA+ JustFloat 绘图模块（适配达妙 HC02 / STM32H723 工程 drv_uart/drv_usb）
 *
 * 说明：
 * - 旧版（F4）工程里 VOFA 往往直接拿 UART 管理对象的 Tx_Buffer + UART_Send_Data()。
 * - HC02 版 drv_uart 只有 RX 双缓冲，不再提供 Tx_Buffer；发送接口为 UART_Transmit_Data()。
 * - 因此本模块改为：内部自带 Tx_Buffer，发送走 UART_Transmit_Data 或 USB_Transmit_Data。
 *
 * JustFloat 帧：float数组(小端) + 4字节帧尾 00 00 80 7F
 */

#ifndef DVC_VOFA_H
#define DVC_VOFA_H

#include <cstdint>
#include <cstring>

#include "1_Middleware/1_Driver/UART/drv_uart.h"
#include "1_Middleware/1_Driver/USB/drv_usb.h"

class Class_Vofa_JustFloat
{
public:
    static constexpr uint8_t MAX_CHANNEL = 10; // 最多支持通道数

    Class_Vofa_JustFloat();

    /**
     * @brief 通过 UART 输出（推荐使用独立调试串口：USART1/USART3...）
     * @param huart         UART 句柄
     * @param channel_num   通道数量(1~MAX_CHANNEL)
     * @param send_period_ms 发送周期(ms)
     */
    void Init_UART(UART_HandleTypeDef *huart, uint8_t channel_num, uint16_t send_period_ms);

    /**
     * @brief 通过 USB CDC 输出（不占用 UART）
     * @param channel_num   通道数量(1~MAX_CHANNEL)
     * @param send_period_ms 发送周期(ms)
     */
    void Init_USB(uint8_t channel_num, uint16_t send_period_ms);

    // 兼容旧接口：Init(huart, ch, period) 等价于 Init_UART
    inline void Init(UART_HandleTypeDef *huart, uint8_t channel_num, uint16_t send_period_ms)
    {
        Init_UART(huart, channel_num, send_period_ms);
    }

    /**
     * @brief 绑定通道到一个 float 变量地址
     */
    void Bind_Channel(uint8_t index, float *p_data);

    /**
     * @brief 1ms 调用一次（放到 TIM7 1ms 任务里）
     */
    void TIM_1ms_PeriodElapsedCallback();

private:
    void Output();

    // 输出通道
    float *Channel_Pointer[MAX_CHANNEL];
    uint8_t Channel_Number;

    // 发送节拍
    uint16_t Send_Period_ms;
    uint16_t Counter_ms;

    // 发送接口
    uint8_t Use_USB; // 0:UART 1:USB
    UART_HandleTypeDef *UART_Handler;

    // JustFloat 一帧最大长度：10*4 + 4 = 44 字节
    uint8_t Tx_Buffer[MAX_CHANNEL * sizeof(float) + 4];
};

#endif // DVC_VOFA_H
