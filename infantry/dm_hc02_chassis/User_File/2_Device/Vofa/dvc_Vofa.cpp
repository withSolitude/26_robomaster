#include "dvc_Vofa.h"

Class_Vofa_JustFloat::Class_Vofa_JustFloat()
    : Channel_Number(0),
      Send_Period_ms(20),
      Counter_ms(0),
      Use_USB(0),
      UART_Handler(nullptr)
{
    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        Channel_Pointer[i] = nullptr;
    }
    memset(Tx_Buffer, 0, sizeof(Tx_Buffer));
}

static inline uint8_t _vofa_clamp_channel(uint8_t ch)
{
    if (ch == 0) return 1;
    if (ch > Class_Vofa_JustFloat::MAX_CHANNEL) return Class_Vofa_JustFloat::MAX_CHANNEL;
    return ch;
}

void Class_Vofa_JustFloat::Init_UART(UART_HandleTypeDef *huart, uint8_t channel_num, uint16_t send_period_ms)
{
    Use_USB = 0;
    UART_Handler = huart;

    Channel_Number = _vofa_clamp_channel(channel_num);
    Send_Period_ms = (send_period_ms == 0) ? 1 : send_period_ms;
    Counter_ms = 0;

    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        Channel_Pointer[i] = nullptr;
    }
}

void Class_Vofa_JustFloat::Init_USB(uint8_t channel_num, uint16_t send_period_ms)
{
    Use_USB = 1;
    UART_Handler = nullptr;

    Channel_Number = _vofa_clamp_channel(channel_num);
    Send_Period_ms = (send_period_ms == 0) ? 1 : send_period_ms;
    Counter_ms = 0;

    for (uint8_t i = 0; i < MAX_CHANNEL; i++)
    {
        Channel_Pointer[i] = nullptr;
    }
}

void Class_Vofa_JustFloat::Bind_Channel(uint8_t index, float *p_data)
{
    if (index >= MAX_CHANNEL)
        return;
    Channel_Pointer[index] = p_data;
}

void Class_Vofa_JustFloat::TIM_1ms_PeriodElapsedCallback()
{
    if (Channel_Number == 0)
        return;
    if (!Use_USB && UART_Handler == nullptr)
        return;

    if (++Counter_ms >= Send_Period_ms)
    {
        Counter_ms = 0;
        Output();
    }
}

void Class_Vofa_JustFloat::Output()
{
    if (Channel_Number == 0)
        return;

    uint8_t *p = Tx_Buffer;

    for (uint8_t i = 0; i < Channel_Number; i++)
    {
        float value = 0.0f;
        if (Channel_Pointer[i] != nullptr)
        {
            value = *Channel_Pointer[i];
        }

        memcpy(p, &value, sizeof(float));
        p += sizeof(float);
    }

    // JustFloat 帧尾: 0x00 0x00 0x80 0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    uint16_t send_len = static_cast<uint16_t>(p - Tx_Buffer);

    if (Use_USB)
    {
        // USB CDC 发送（若忙会返回非 0，可选择忽略/重发；这里直接丢帧）
        (void)USB_Transmit_Data(Tx_Buffer, send_len);
    }
    else
    {
        // UART DMA 发送（若 BUSY 直接丢帧，避免卡死）
        if (UART_Handler != nullptr)
        {
            (void)UART_Transmit_Data(UART_Handler, Tx_Buffer, send_len);
        }
    }
}
