#include "dvc_Vofa_Tuner.h"

#include <cctype>
#include <cstdlib>

Class_Vofa_Tuner *Class_Vofa_Tuner::Instance = nullptr;

void Class_Vofa_Tuner::Init(UART_HandleTypeDef *__huart, uint16_t __rx_len)
{
    (void)__rx_len; // HC02 drv_uart 固定 UART_BUFFER_SIZE，保留该参数仅为兼容旧调用

    Use_USB = 0;
    UART_Handler = __huart;
    Instance = this;

    Float_Item_Num = 0;
    PID_Item_Num = 0;
    Line_Len = 0;
    memset(Line_Buf, 0, sizeof(Line_Buf));

    // 开启串口 DMA 接收 + 回调（HC02 版接口没有 rx_len 参数）
    UART_Init(__huart, Static_UART_Callback);
}

void Class_Vofa_Tuner::Init_USB()
{
    Use_USB = 1;
    UART_Handler = nullptr;
    Instance = this;

    Float_Item_Num = 0;
    PID_Item_Num = 0;
    Line_Len = 0;
    memset(Line_Buf, 0, sizeof(Line_Buf));
}

uint8_t Class_Vofa_Tuner::Bind_Float(uint16_t __id, float *__p_float)
{
    if (__p_float == nullptr) return 0;
    if (Float_Item_Num >= MAX_FLOAT_ITEM) return 0;

    Float_Item[Float_Item_Num].id = __id;
    Float_Item[Float_Item_Num].p = __p_float;
    Float_Item_Num++;
    return 1;
}

uint8_t Class_Vofa_Tuner::Bind_PID(uint16_t __base_id, Class_PID *__pid)
{
    if (__pid == nullptr) return 0;
    if (PID_Item_Num >= MAX_PID_ITEM) return 0;

    PID_Item[PID_Item_Num].base_id = __base_id;
    PID_Item[PID_Item_Num].pid = __pid;
    PID_Item_Num++;
    return 1;
}

void Class_Vofa_Tuner::Static_UART_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Instance != nullptr)
    {
        Instance->UART_RxCpltCallback(Buffer, Length);
    }
}

void Class_Vofa_Tuner::UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Buffer == nullptr || Length == 0) return;
    for (uint16_t i = 0; i < Length; i++)
    {
        Parse_Byte(Buffer[i]);
    }
}

void Class_Vofa_Tuner::USB_RxCpltCallback(uint8_t *Buffer, uint16_t Length)
{
    // USB 模式与 UART 模式共用解析逻辑
    UART_RxCpltCallback(Buffer, Length);
}

void Class_Vofa_Tuner::Parse_Byte(uint8_t ch)
{
    if (ch == '\n' || ch == '\r')
    {
        if (Line_Len > 0)
        {
            if (Line_Len >= LINE_BUF_LEN) Line_Len = LINE_BUF_LEN - 1;
            Line_Buf[Line_Len] = '\0';
            Handle_Line(Line_Buf);
        }
        Line_Len = 0;
        memset(Line_Buf, 0, sizeof(Line_Buf));
        return;
    }

    // 过滤不可见字符
    if (ch < 0x20 || ch > 0x7E) return;

    if (Line_Len < (LINE_BUF_LEN - 1))
    {
        Line_Buf[Line_Len++] = static_cast<char>(ch);
    }
    else
    {
        // 超长：丢弃本行
        Line_Len = 0;
        memset(Line_Buf, 0, sizeof(Line_Buf));
    }
}

void Class_Vofa_Tuner::Handle_Line(const char *line)
{
    if (line == nullptr) return;

    // 跳过前导空格
    while (*line != 0 && isspace((int)(unsigned char)*line)) line++;
    if (*line == 0) return;

    // 1) 找第一个整数（id）
    const char *p = line;
    while (*p != 0 && !(isdigit((int)(unsigned char)*p) || *p == '-')) p++;
    if (*p == 0) return;

    char *endp = nullptr;
    long id_l = strtol(p, &endp, 10);
    if (endp == p) return;

    // 2) 找后续第一个 float（value）
    p = endp;
    while (*p != 0 && (*p == ',' || *p == ':' || *p == '=' || isspace((int)(unsigned char)*p))) p++;
    while (*p != 0 && !(isdigit((int)(unsigned char)*p) || *p == '-' || *p == '.')) p++;
    if (*p == 0) return;

    char *endp2 = nullptr;
    float value = strtof(p, &endp2);
    if (endp2 == p) return;

    Apply_Command((uint16_t)id_l, value);
}

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
        if (pid == nullptr) continue;
        if (id < base) continue;

        uint16_t offset = id - base;
        switch (offset)
        {
        case Vofa_Tuner_PID_KP: pid->Set_K_P(value); break;
        case Vofa_Tuner_PID_KI: pid->Set_K_I(value); break;
        case Vofa_Tuner_PID_KD: pid->Set_K_D(value); break;
        case Vofa_Tuner_PID_KF: pid->Set_K_F(value); break;
        case Vofa_Tuner_PID_I_MAX: pid->Set_I_Out_Max(value); break;
        case Vofa_Tuner_PID_OUT_MAX: pid->Set_Out_Max(value); break;
        default: continue;
        }

        if (Reset_Integral_On_Tune)
        {
            pid->Set_Integral_Error(0.0f);
        }
        return 1;
    }

    return 0;
}
