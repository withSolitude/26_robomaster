//
// Created by Lenovo on 2026/3/2.
//

#ifndef TEST_ROBOWAKER_DVC_DM_IMU_H
#define TEST_ROBOWAKER_DVC_DM_IMU_H

#include "stm32h7xx_hal.h"
#include "1_Middleware/1_Driver/CAN/drv_can.h"

#define ACCEL_CAN_MAX           (235.2f)
#define ACCEL_CAN_MIN           (-235.2f)
#define GYRO_CAN_MAX            (34.88f)
#define GYRO_CAN_MIN            (-34.88f)
#define PITCH_CAN_MAX           (90.0f)
#define PITCH_CAN_MIN           (-90.0f)
#define ROLL_CAN_MAX            (180.0f)
#define ROLL_CAN_MIN            (-180.0f)
#define YAW_CAN_MAX             (180.0f)
#define YAW_CAN_MIN             (-180.0f)
#define Quaternion_CAN_MIN      (-1.0f)
#define Quaternion_CAN_MAX      (1.0f)

enum Enum_DM_IMU_Status
{
    Enum_DM_IMU_Status_DISABLE = 0,
    Enum_DM_IMU_Status_ENABLE,
};

class Class_DM_IMU
{
public:

    enum Enum_DM_IMU_Request_Type : uint8_t
    {
        Enum_DM_IMU_Request_Type_NONE  = 0,
        Enum_DM_IMU_Request_Type_ACCEL = 1,
        Enum_DM_IMU_Request_Type_GYRO  = 2,
        Enum_DM_IMU_Request_Type_EULER = 3,
    };

    void Disable_Active_Output(bool save = false);
    void TIM_1ms_Poll_PeriodElapsedCallback();

    inline uint32_t Get_Rx_Tick();
    inline uint8_t  Get_Pending_Request();

    void Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID, uint16_t __MST_ID);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void CAN_RxCpltCallback(const uint8_t *Rx_Data);

    // ------------ 配置/控制（按达妙例程）------------
    void Set_CAN_ID(uint16_t can_id) { CAN_ID = can_id; }      // 配置帧发送目标ID（上位机CANID）
    void Set_MST_ID(uint16_t mst_id) { CAN_Rx_ID = mst_id; }   // 数据帧接收ID（上位机MSTID）

    // 写寄存器（0xCC ... 0xDD）
    // reg: 寄存器号；rw: 0=read, 1=write；data: 32bit，小端
    void Write_Reg(uint8_t reg, uint32_t data);
    void Read_Reg(uint8_t reg);

    // 一键切到“主动发送模式”（建议在 Task_Init() 调用一次）
    void Enable_Active_Output(uint16_t period_ms, bool save = true);

    inline float Get_Acc_X();

    inline float Get_Acc_Y();

    inline float Get_Acc_Z();

    inline float Get_Gyro_X();

    inline float Get_Gyro_Y();

    inline float Get_Gyro_Z();

    inline float Get_Pitch_Deg();

    inline float Get_Yaw_Deg();

    inline float Get_Roll_Deg();

    inline Enum_DM_IMU_Status  Get_IMU_Status();

    inline float Get_Last_ID();

    inline float Get_Last_Raw0();

    inline float Get_Last_Raw1();

    inline float Get_Last_Raw2();

    inline float Get_Last_Raw3();

    inline float Get_Last_Raw4();

    inline float Get_Last_Raw5();

    inline float Get_Last_Raw6();

    inline float Get_Last_Raw7();

    void Request_Accel();
    void Request_Gyro();
    void Request_Euler();

private:

    Enum_DM_IMU_Request_Type Pending_Request = Enum_DM_IMU_Request_Type_NONE;

    uint8_t  Poll_Index = 0;          // 0->ACCEL, 1->GYRO, 2->EULER
    uint16_t Poll_Interval_ms = 5;    // 每 5ms 发一次请求
    uint16_t Poll_Tick = 0;

    uint16_t Wait_Timeout_ms = 10;    // 发出后 10ms 没回就重发
    uint16_t Wait_Tick = 0;

    void Send_Request(Enum_DM_IMU_Request_Type req);

    //内部变量

    Enum_DM_IMU_Status imu_status = Enum_DM_IMU_Status_DISABLE;
    //
    Struct_CAN_Manage_Object  *CAN_Manage_Object = nullptr;

    uint16_t CAN_ID = 0;   // 配置帧发送目标ID（CANID）

    uint16_t CAN_Rx_ID = 0;
    // 上一次接收到的时间
    uint32_t Last_Rx_Tick = 0;
    // 接收到的时间
    uint32_t Rx_Tick = 0;
    //

    // imu温度
    float Temperature = 0.0f;
    // Acc_X轴映射值
    float Acc_X = 0.0f;
    // Acc_Y轴映射值
    float Acc_Y = 0.0f;
    // Acc_Z轴映射值
    float Acc_Z = 0.0f;
    // Gyro_X轴映射值
    float Gyro_X = 0.0f;
    // Gryo_Y轴映射值
    float Gyro_Y = 0.0f;
    // Groy_Z轴映射值
    float Gyro_Z = 0.0f;
    // Pitch映射值
    float Pitch_Deg = 0.0f;
    // Yaw映射值
    float Yaw_Deg = 0.0f;
    // Roll映射值
    float Roll_Deg = 0.0f;
    // 四元数数据
    float Quart[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    uint16_t Last_ID = 0;
    uint8_t Last_Raw[8] = {0};


    // 内部函数

    void UpdateTemperature(const uint8_t* pData);

    void UpdateAccel(const uint8_t* pData);

    void UpdateGyro(const uint8_t* pData);

    void UpdateEuler(const uint8_t* pData);

    void UpdateQuaternion(const uint8_t* pData);

    void Data_Process(const uint8_t* pData);



};

/**
 * @brief 获取Acc_X的值
 *
 * @return Acc_X
 */
inline float Class_DM_IMU::Get_Acc_X()
{
    return Acc_X;
}

/**
 * @brief 获取Acc_Y的值
 *
 * @return Acc_Y
 */
inline float Class_DM_IMU::Get_Acc_Y()
{
    return Acc_Y;
}

/**
 * @brief 获取Acc_Z的值
 *
 * @return Acc_Z
 */
inline float Class_DM_IMU::Get_Acc_Z()
{
    return Acc_Z;
}

/**
 * @brief 获取Gyro_X的值
 *
 * @return Gyro_X
 */
inline float Class_DM_IMU::Get_Gyro_X()
{
    return Gyro_X;
}

/**
 * @brief 获取Gyro_Y的值
 *
 * @return Gyro_Y
 */
inline float Class_DM_IMU::Get_Gyro_Y()
{
    return Gyro_Y;
}

/**
 * @brief 获取Gyro_Z的值
 *
 * @return Gyro_Z
 */
inline float Class_DM_IMU::Get_Gyro_Z()
{
    return Gyro_Z;
}

/**
 * @brief 获取Pitch的值
 *
 * @return Pitch_deg
 */
inline float Class_DM_IMU::Get_Pitch_Deg()
{
    return Pitch_Deg;
}

/**
 * @brief 获取Yaw的值
 *
 * @return Yaw_Deg
 */
inline float Class_DM_IMU::Get_Yaw_Deg()
{
    return Yaw_Deg;
}

/**
 * @brief 获取PRoll的值
 *
 * @return Roll_Deg
 */
inline float Class_DM_IMU::Get_Roll_Deg()
{
    return Roll_Deg;
}

/**
 * @brief 获取IMU状态
 *
 */
inline Enum_DM_IMU_Status  Class_DM_IMU::Get_IMU_Status()
{
   return  imu_status;
}


inline float Class_DM_IMU::Get_Last_ID()
{
    return float(Last_ID);
}

inline float Class_DM_IMU::Get_Last_Raw0()
{
    return float(Last_Raw[0]);
}
inline float Class_DM_IMU::Get_Last_Raw1()
{
    return float(Last_Raw[1]);
}
inline float Class_DM_IMU::Get_Last_Raw2()
{
    return float(Last_Raw[2]);
}
inline float Class_DM_IMU::Get_Last_Raw3()
{
    return float(Last_Raw[3]);
}
inline float Class_DM_IMU::Get_Last_Raw4()
{
    return float(Last_Raw[4]);
}
inline float Class_DM_IMU::Get_Last_Raw5()
{
    return float(Last_Raw[5]);
}
inline float Class_DM_IMU::Get_Last_Raw6()
{
    return float(Last_Raw[6]);
}
inline float Class_DM_IMU::Get_Last_Raw7()
{
    return float(Last_Raw[7]);
}

inline uint32_t Class_DM_IMU::Get_Rx_Tick()
{
    return Rx_Tick;
}

inline uint8_t Class_DM_IMU::Get_Pending_Request()
{
    return static_cast<uint8_t>(Pending_Request);
}
#endif //TEST_ROBOWAKER_DVC_DM_IMU_H