#ifndef TEST_ROBOWAKER_DVC_AUTOAIM_H
#define TEST_ROBOWAKER_DVC_AUTOAIM_H

#include "1_Middleware/1_Driver/USB/drv_usb.h"
#include <stdint.h>

#define AUTOAIM_FRAME_HEAD_0   ((uint8_t)'S')
#define AUTOAIM_FRAME_HEAD_1   ((uint8_t)'P')

#define AUTOAIM_TX_FRAME_LEN   43
#define AUTOAIM_RX_FRAME_LEN   29
#define AUTOAIM_PARSE_BUFFER_SIZE 1024

// 视觉数据基础安全边界
#define AUTOAIM_YAW_ABS_MAX       1000.0f
#define AUTOAIM_PITCH_ABS_MAX     2.0f

enum Enum_AutoAim_Status
{
    AutoAim_Status_DISABLE = 0,
    AutoAim_Status_ENABLE,
};

enum Enum_AutoAim_Mode : uint8_t
{
    AutoAim_Mode_IDLE = 0,
    AutoAim_Mode_CONTROL_NO_FIRE = 1,
    AutoAim_Mode_CONTROL_FIRE = 2,
};

// 下位机 -> 上位机
struct Struct_AutoAim_USB_Tx_Data
{
    uint8_t head[2];
    uint8_t mode;

    float q[4];

    float yaw;
    float yaw_vel;

    float pitch;
    float pitch_vel;

    float bullet_speed;
    uint16_t bullet_count;

    uint16_t crc16;
} __attribute__((packed));

// 上位机 -> 下位机
struct Struct_AutoAim_USB_Rx_Data
{
    uint8_t head[2];
    uint8_t mode;

    float yaw;
    float yaw_vel;
    float yaw_acc;

    float pitch;
    float pitch_vel;
    float pitch_acc;

    uint16_t crc16;
} __attribute__((packed));

class Class_AutoAim
{
public:
    void Init();

    void USB_RxCpltCallback(uint8_t *Rx_Data, uint16_t Length);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_10ms_Send_PeriodElapsedCallback();

    inline Enum_AutoAim_Status Get_Status();

    inline uint8_t Get_Rx_Mode();

    inline float Get_Target_Yaw();

    inline float Get_Target_Pitch();

    inline float Get_Target_Yaw_Vel();

    inline float Get_Target_Pitch_Vel();

    inline float Get_Target_Yaw_Acc();

    inline float Get_Target_Pitch_Acc();

    inline void Set_Tx_Mode(uint8_t __mode);

    inline void Set_Quaternion(const float *__q);

    inline void Set_Now_Yaw(float __yaw);

    inline void Set_Now_Yaw_Vel(float __yaw_vel);

    inline void Set_Now_Pitch(float __pitch);

    inline void Set_Now_Pitch_Vel(float __pitch_vel);

    inline void Set_Bullet_Speed(float __bullet_speed);

    inline void Set_Bullet_Count(uint16_t __bullet_count);

public:
    uint32_t Header_Cnt = 0;
    uint32_t CRC_Ok_Cnt = 0;
    uint32_t CRC_Fail_Cnt = 0;
    uint32_t Send_Ok_Cnt = 0;
    uint32_t Send_Fail_Cnt = 0;

protected:
    Enum_AutoAim_Status AutoAim_Status = AutoAim_Status_DISABLE;

    Struct_AutoAim_USB_Tx_Data Tx_Data{};
    Struct_AutoAim_USB_Rx_Data Rx_Data{};

    uint8_t Parse_Buffer[AUTOAIM_PARSE_BUFFER_SIZE] = {0};
    uint16_t Parse_Len = 0;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;
    uint16_t Offline_Count = 0;

    float Target_Yaw = 0.0f;
    float Target_Pitch = 0.0f;

protected:
    bool Data_Process(uint8_t *Rx_Data, uint16_t Length);

    bool Frame_Data_Is_Valid(const Struct_AutoAim_USB_Rx_Data &data);

    void Reset_Rx_Data_To_Idle();

    void Output();

    uint16_t Calc_CRC16(const uint8_t *data, uint16_t len);

    bool Verify_CRC16(const uint8_t *data, uint16_t len);

    void Append_CRC16(uint8_t *data, uint16_t len);
};

inline Enum_AutoAim_Status Class_AutoAim::Get_Status()
{
    return AutoAim_Status;
}

inline uint8_t Class_AutoAim::Get_Rx_Mode()
{
    return Rx_Data.mode;
}

inline float Class_AutoAim::Get_Target_Yaw()
{
    return Target_Yaw;
}

inline float Class_AutoAim::Get_Target_Pitch()
{
    return Target_Pitch;
}

inline float Class_AutoAim::Get_Target_Yaw_Vel()
{
    return Rx_Data.yaw_vel;
}

inline float Class_AutoAim::Get_Target_Pitch_Vel()
{
    return Rx_Data.pitch_vel;
}

inline float Class_AutoAim::Get_Target_Yaw_Acc()
{
    return Rx_Data.yaw_acc;
}

inline float Class_AutoAim::Get_Target_Pitch_Acc()
{
    return Rx_Data.pitch_acc;
}

inline void Class_AutoAim::Set_Tx_Mode(uint8_t __mode)
{
    if (__mode > AutoAim_Mode_CONTROL_FIRE)
    {
        __mode = AutoAim_Mode_IDLE;
    }

    Tx_Data.mode = __mode;
}

inline void Class_AutoAim::Set_Quaternion(const float *__q)
{
    if (__q == nullptr)
    {
        Tx_Data.q[0] = 1.0f;
        Tx_Data.q[1] = 0.0f;
        Tx_Data.q[2] = 0.0f;
        Tx_Data.q[3] = 0.0f;
        return;
    }

    Tx_Data.q[0] = __q[0];
    Tx_Data.q[1] = __q[1];
    Tx_Data.q[2] = __q[2];
    Tx_Data.q[3] = __q[3];
}

inline void Class_AutoAim::Set_Now_Yaw(float __yaw)
{
    Tx_Data.yaw = __yaw;
}

inline void Class_AutoAim::Set_Now_Yaw_Vel(float __yaw_vel)
{
    Tx_Data.yaw_vel = __yaw_vel;
}

inline void Class_AutoAim::Set_Now_Pitch(float __pitch)
{
    Tx_Data.pitch = __pitch;
}

inline void Class_AutoAim::Set_Now_Pitch_Vel(float __pitch_vel)
{
    Tx_Data.pitch_vel = __pitch_vel;
}

inline void Class_AutoAim::Set_Bullet_Speed(float __bullet_speed)
{
    Tx_Data.bullet_speed = __bullet_speed;
}

inline void Class_AutoAim::Set_Bullet_Count(uint16_t __bullet_count)
{
    Tx_Data.bullet_count = __bullet_count;
}

#endif