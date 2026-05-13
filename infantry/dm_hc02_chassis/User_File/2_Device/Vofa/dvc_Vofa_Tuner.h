/**
 * @file dvc_Vofa_Tuner.h
 * @brief VOFA+ Slider/Widget 在线调参（适配 HC02 drv_uart/drv_usb）
 *
 * 推荐发送格式（ASCII）：
 *   <id>,<value>\n
 * 例：
 *   1000,0.30\n        -> id=1000 value=0.30
 *   id=1100 val=0.02\n  -> 也支持(自动提取数字)
 *
 * 绑定 PID：Bind_PID(base_id, pid)
 *  - base+0 KP
 *  - base+1 KI
 *  - base+2 KD
 *  - base+3 KF
 *  - base+4 I_Out_Max
 *  - base+5 Out_Max
 */

#ifndef DVC_VOFA_TUNER_H
#define DVC_VOFA_TUNER_H

#include <cstdint>
#include <cstring>

#include "1_Middleware/1_Driver/UART/drv_uart.h"
#include "1_Middleware/1_Driver/USB/drv_usb.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"

enum Enum_Vofa_Tuner_PID_Param_Offset
{
    Vofa_Tuner_PID_KP = 0,
    Vofa_Tuner_PID_KI = 1,
    Vofa_Tuner_PID_KD = 2,
    Vofa_Tuner_PID_KF = 3,
    Vofa_Tuner_PID_I_MAX = 4,
    Vofa_Tuner_PID_OUT_MAX = 5,
};

class Class_Vofa_Tuner
{
public:
    /**
     * @brief 通过 UART 初始化（建议与 VOFA 绘图同一路）
     * @note  __rx_len 为兼容旧接口保留，在 HC02 drv_uart 中会被忽略
     */
    void Init(UART_HandleTypeDef *__huart, uint16_t __rx_len);

    // 更直观的名字
    inline void Init_UART(UART_HandleTypeDef *__huart) { Init(__huart, 0); }

    /**
     * @brief USB 模式初始化（不占 UART）
     * @note  USB 收包需要你在 Serial_USB_Call_Back 里调用 USB_RxCpltCallback
     */
    void Init_USB();

    uint8_t Bind_Float(uint16_t __id, float *__p_float);
    uint8_t Bind_PID(uint16_t __base_id, Class_PID *__pid);

    inline void Set_Reset_Integral_On_Tune(uint8_t __en) { Reset_Integral_On_Tune = __en; }

    // drv_uart 回调
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);
    // USB 回调（由你的 USB 回调函数转发进来）
    void USB_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

private:
    struct Struct_Vofa_Tuner_Float_Item
    {
        uint16_t id;
        float *p;
    };

    struct Struct_Vofa_Tuner_PID_Item
    {
        uint16_t base_id;
        Class_PID *pid;
    };

    static void Static_UART_Callback(uint8_t *Buffer, uint16_t Length);

    void Parse_Byte(uint8_t ch);
    void Handle_Line(const char *line);
    uint8_t Apply_Command(uint16_t id, float value);

    UART_HandleTypeDef *UART_Handler = nullptr;
    uint8_t Use_USB = 0;

    // 单实例（工程习惯）
    static Class_Vofa_Tuner *Instance;

    static const uint8_t MAX_FLOAT_ITEM = 32;
    static const uint8_t MAX_PID_ITEM = 16;
    Struct_Vofa_Tuner_Float_Item Float_Item[MAX_FLOAT_ITEM];
    uint8_t Float_Item_Num = 0;
    Struct_Vofa_Tuner_PID_Item PID_Item[MAX_PID_ITEM];
    uint8_t PID_Item_Num = 0;

    // 拼行缓存
    static const uint16_t LINE_BUF_LEN = 96;
    char Line_Buf[LINE_BUF_LEN];
    uint16_t Line_Len = 0;

    uint8_t Reset_Integral_On_Tune = 0;
};

#endif // DVC_VOFA_TUNER_H
