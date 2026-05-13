//
// Created by Lenovo on 2025/12/10.
//

#ifndef TEST_ROBOWAKER_DVC_VOFA_H
#define TEST_ROBOWAKER_DVC_VOFA_H

#include "UART/drv_uart.h"
#include <stdint.h>
/**
 * 向串口发送一帧字节数据的回调函数类型
 *
 *可以写一个函数：
 *  void UART_2_Send(uint8_t *buf,uint16_t len)
 *  {
 *      HAL_UART_Transmit(&huart2,buf,len,100);
 *  }
 * 然后Init的时候把这个函数指针传进来就可以了
 *
 */
typedef void (*VOFA_Send_Func)(uint8_t *data,uint16_t length);


/**
 * justfloat 协议
 *
 *
 *  本协议是小端浮点数组形式的字节流协议，纯十六进制浮点传输，
 *  节省带宽。此协议非常适合用在通道数量多、
 *  发送频率高的时候。
 *
 *
 *数据格式
 *
*  #define CH_COUNT 通道数量
    struct Frame {
        float fdata[CH_COUNT];
        unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
    };
*fdata为小端浮点数组，里面放着需要发送的CH_COUNT个数据。
*tail为帧尾
*
*/
class Class_Vofa_JustFloat
{
    public:

    void Init(UART_HandleTypeDef *huart,uint8_t channel_num,uint16_t send_period_ms);

    void Bind_Channel(uint8_t index,float *p_data);

    void TIM_1ms_PeriodElapsedCallback();

    static constexpr uint8_t MAX_CHANNEL = 10;          //最多支持通道数

    Class_Vofa_JustFloat();

    private:

    void Output();

    Struct_UART_Manage_Object *UART_Manage_Object;                           //发送函数
    float *Channel_Pointer[MAX_CHANNEL];                //通道指针
    uint8_t Channel_Number;                             //通道数量

    uint16_t Send_Period_ms;                            //发送周期
    uint16_t Counter_ms;                                //1ms计数器
};


#endif //TEST_ROBOWAKER_DVC_VOFA_H