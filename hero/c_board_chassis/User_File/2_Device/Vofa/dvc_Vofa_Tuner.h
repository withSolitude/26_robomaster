//
// Created by Lenovo on 2026/02/20.
//

#ifndef TEST_ROBOWAKER_DVC_VOFA_TUNER_H
#define TEST_ROBOWAKER_DVC_VOFA_TUNER_H

#include <cstdint>

#include "UART/drv_uart.h"
#include "2_Algorithm/PID/alg_pid.h"

/*
 * VOFA+ Slider/Widget 在线调参模块
 *
 * 推荐发送格式(ASCII)：
 *   <id>,<value>\n
 * 例：
 *   100,0.30\n        -> id=100 value=0.30
 *   id=110 val=0.02\n  -> 也支持(会自动提取数字)
 *
 * 典型用法：
 *   Class_Vofa_Tuner Vofa_Tuner;
 *   Vofa_Tuner.Init(&huart1, 64);
 *   Vofa_Tuner.Bind_PID(100, &Chassis.PID_Vx);    // 100:KP 101:KI 102:KD 103:Imax 104:OutMax 105:OutMin 106:Deadzone
 */

// PID 参数映射(以 Bind_PID(base_id, pid) 为基准)
enum Enum_Vofa_Tuner_PID_Param_Offset
{
    Vofa_Tuner_PID_KP       = 0,
    Vofa_Tuner_PID_KI       = 1,
    Vofa_Tuner_PID_KD       = 2,
    Vofa_Tuner_PID_KF       = 3,
    Vofa_Tuner_PID_I_MAX    = 4,
    Vofa_Tuner_PID_OUT_MAX  = 5,
};

class Class_Vofa_Tuner
{
public:

    void Init(UART_HandleTypeDef *__huart, uint16_t __rx_len);

    // 绑定一个 float 变量：收到对应 id 后，直接写入 *p
    // 注意：变量应为全局/静态/长期有效(不要绑局部变量地址)
    uint8_t Bind_Float(uint16_t __id, float *__p_float);

    // 绑定 PID：base_id + offset 对应不同参数(见 Enum_Vofa_Tuner_PID_Param_Offset)
    uint8_t Bind_PID(uint16_t __base_id, Class_PID *__pid);

    // 可选：参数变化后是否清积分，默认不清
    inline void Set_Reset_Integral_On_Tune(uint8_t __en);

    // 串口接收回调(由 drv_uart 调用)
    void UART_RxCpltCallback(uint8_t *Buffer, uint16_t Length);

protected:

    typedef struct
    {
        uint16_t id;
        float *p;
    } Struct_Vofa_Tuner_Float_Item;

    typedef struct
    {
        uint16_t base_id;
        Class_PID *pid;
    } Struct_Vofa_Tuner_PID_Item;

    static void Static_UART_Callback(uint8_t *Buffer, uint16_t Length);

    void Parse_Byte(uint8_t ch);
    void Handle_Line(const char *line);
    uint8_t Apply_Command(uint16_t id, float value);

    static float Constrain_Float(float x, float lo, float hi);

    UART_HandleTypeDef *UART_Handler = nullptr;

    // 单实例(当前工程习惯)
    static Class_Vofa_Tuner *Instance;

    // 绑定表
    static const uint8_t MAX_FLOAT_ITEM = 32;
    static const uint8_t MAX_PID_ITEM   = 16;

    Struct_Vofa_Tuner_Float_Item Float_Item[MAX_FLOAT_ITEM];
    uint8_t Float_Item_Num = 0;

    Struct_Vofa_Tuner_PID_Item PID_Item[MAX_PID_ITEM];
    uint8_t PID_Item_Num = 0;

    // 收包缓存(逐字节拼行)
    static const uint16_t LINE_BUF_LEN = 96;
    char Line_Buf[LINE_BUF_LEN];
    uint16_t Line_Len = 0;

    uint8_t Reset_Integral_On_Tune = 0;
};

inline void Class_Vofa_Tuner::Set_Reset_Integral_On_Tune(uint8_t __en)
{
    Reset_Integral_On_Tune = __en;
}

#endif //TEST_ROBOWAKER_DVC_VOFA_TUNER_H
