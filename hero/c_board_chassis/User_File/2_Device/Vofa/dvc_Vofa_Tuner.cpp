//
// Created by Lenovo on 2026/02/20.
//

#include "dvc_vofa_tuner.h"

#include <cctype>
#include <cstdlib>
#include <cstring>

Class_Vofa_Tuner *Class_Vofa_Tuner::Instance = nullptr;

/**
 * @brief 初始化 VOFA 调参模块
 *
 * @param __huart  使用的串口(建议与 VOFA 绘图同一路)
 * @param __rx_len DMA 接收长度(推荐 64)
 */
void Class_Vofa_Tuner::Init(UART_HandleTypeDef *__huart, uint16_t __rx_len)
{
    UART_Handler = __huart;

    // 单实例(如需多实例可改为“按 UART 句柄查表”)。
    Instance = this;

    Line_Len = 0;
    memset(Line_Buf, 0, sizeof(Line_Buf));

    // 开启串口 DMA 接收 + 回调
    UART_Init(__huart, Static_UART_Callback, __rx_len);
}

/**
 * @brief 绑定 float 变量
 */
uint8_t Class_Vofa_Tuner::Bind_Float(uint16_t __id, float *__p_float)
{
    if (__p_float == nullptr)
    {
        return 0;
    }

    if (Float_Item_Num >= MAX_FLOAT_ITEM)
    {
        return 0;
    }

    Float_Item[Float_Item_Num].id = __id;
    Float_Item[Float_Item_Num].p  = __p_float;
    Float_Item_Num++;

    return 1;
}

/**
 * @brief 绑定 PID 参数(基准 id)
 */
uint8_t Class_Vofa_Tuner::Bind_PID(uint16_t __base_id, Class_PID *__pid)
{
    if (__pid == nullptr)
    {
        return 0;
    }

    if (PID_Item_Num >= MAX_PID_ITEM)
    {
        return 0;
    }

    PID_Item[PID_Item_Num].base_id = __base_id;
    PID_Item[PID_Item_Num].pid     = __pid;
    PID_Item_Num++;

    return 1;
}

/**
 * @brief drv_uart 的静态回调
 */
void Class_Vofa_Tuner::Static_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Instance != nullptr)
    {
        Instance->UART_RxCpltCallback(Buffer, Length);
    }
}

/**
 * @brief 串口接收回调
 */
void Class_Vofa_Tuner::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr || Length == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < Length; i++)
    {
        Parse_Byte(Buffer[i]);
    }
}

/**
 * @brief 按字节拼行，遇到 \r/\n 处理一行
 */
void Class_Vofa_Tuner::Parse_Byte(uint8_t ch)
{
    // 行结束
    if (ch == '\n' || ch == '\r')
    {
        if (Line_Len > 0)
        {
            if (Line_Len >= LINE_BUF_LEN)
            {
                Line_Len = LINE_BUF_LEN - 1;
            }
            Line_Buf[Line_Len] = '\0';
            Handle_Line(Line_Buf);
        }

        Line_Len = 0;
        memset(Line_Buf, 0, sizeof(Line_Buf));
        return;
    }

    // 过滤不可见字符
    if (ch < 0x20 || ch > 0x7E)
    {
        return;
    }

    if (Line_Len < (LINE_BUF_LEN - 1))
    {
        Line_Buf[Line_Len++] = (char)ch;
    }
    else
    {
        // 超长：丢弃本行
        Line_Len = 0;
        memset(Line_Buf, 0, sizeof(Line_Buf));
    }
}

/**
 * @brief 处理一行命令：提取 id 和 value
 */
void Class_Vofa_Tuner::Handle_Line(const char *line)
{
    if (line == nullptr)
    {
        return;
    }

    // 允许前后空格
    while (*line != 0 && isspace((int)(unsigned char)*line))
    {
        line++;
    }

    if (*line == 0)
    {
        return;
    }

    // 1) 找到第一个整数(id)
    const char *p = line;
    while (*p != 0 && !(isdigit((int)(unsigned char)*p) || *p == '-'))
    {
        p++;
    }

    if (*p == 0)
    {
        return;
    }

    char *endp = nullptr;
    long id_l = strtol(p, &endp, 10);
    if (endp == p)
    {
        return;
    }

    // 2) 找到后续的第一个 float(value)
    p = endp;

    // 先跳过常见分隔符
    while (*p != 0 && (*p == ',' || *p == ':' || *p == '=' || isspace((int)(unsigned char)*p)))
    {
        p++;
    }

    // 如果不是数字/符号，继续找下一个数字片段(兼容 "id=100,val=0.3")
    while (*p != 0 && !(isdigit((int)(unsigned char)*p) || *p == '-' || *p == '.'))
    {
        p++;
    }

    if (*p == 0)
    {
        return;
    }

    char *endp2 = nullptr;
    float value = strtof(p, &endp2);
    if (endp2 == p)
    {
        return;
    }

    // 应用
    Apply_Command((uint16_t)id_l, value);
}

/**
 * @brief 将 id/value 写到绑定对象
 */
uint8_t Class_Vofa_Tuner::Apply_Command(uint16_t id, float value)
{
    // 1) float 直写
    for (uint8_t i = 0; i < Float_Item_Num; i++)
    {
        if (Float_Item[i].id == id)
        {
            *(Float_Item[i].p) = value;
            return 1;
        }
    }

    // 2) PID 映射
    for (uint8_t i = 0; i < PID_Item_Num; i++)
    {
        uint16_t base = PID_Item[i].base_id;
        Class_PID *pid = PID_Item[i].pid;

        if (pid == nullptr)
        {
            continue;
        }

        if (id < base)
        {
            continue;
        }

        uint16_t offset = id - base;

        switch (offset)
        {
        case (Vofa_Tuner_PID_KP):
            {
                pid->Set_K_P(value);
                break;
            }

        case (Vofa_Tuner_PID_KI):
            {
                pid->Set_K_I(value);
                break;
            }

        case (Vofa_Tuner_PID_KD):
            {
                pid->Set_K_D(value);
                break;
            }

        case (Vofa_Tuner_PID_KF):
            {
                pid->Set_K_F(value);
                break;
            }

        case (Vofa_Tuner_PID_I_MAX):
            {
                // 你的 PID 类里是这个名字
                pid->Set_I_Out_Max(value);
                break;
            }

        case (Vofa_Tuner_PID_OUT_MAX):
            {
                pid->Set_Out_Max(value);
                break;
            }

        default:
            {
                continue;
            }
        }

        if (Reset_Integral_On_Tune != 0)
        {
            pid->Set_Integral_Error(0.0f);
        }

        return 1;
    }

    return 0;
}

float Class_Vofa_Tuner::Constrain_Float(float x, float lo, float hi)
{
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    return x;
}
