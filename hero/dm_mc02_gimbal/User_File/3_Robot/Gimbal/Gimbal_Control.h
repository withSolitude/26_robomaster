//
// Created by Lenovo on 2026/1/13.
//

#ifndef TEST_ROBOWAKER_GIMBAL_CONTROL_H
#define TEST_ROBOWAKER_GIMBAL_CONTROL_H

#include <cmath>

#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/1_Driver/Math/drv_math.h"
#include "2_Device/BMI088/imu.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/DM_IMU/dvc_dm_imu.h"
#include "1_Middleware/2_Algorithm/Filter/Frequency/alg_filter_frequency.h"

enum Enum_Gimbal_Control_Type
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_POSITION,
};

// 上电自举状态机：等IMU -> 抬头进安全区 -> 正常控制
enum Enum_Gimbal_Startup_State
{
    Gimbal_Startup_WAIT_IMU = 0,
    Gimbal_Startup_LIFT_TO_SAFE,
    Gimbal_Startup_NORMAL,
};

class Class_Gimbal
{
public:
    Class_DM_IMU DM_IMU;
    Class_Board_IMU Board_IMU;

    Class_Motor_DM_Normal Yaw_Motor_MIT;
    Class_Motor_DM_Normal Pitch_Motor_MIT;

    Class_PID Class_PID_Yaw_Angle_IMU;
    Class_PID Class_PID_Yaw_Omega_IMU;
    Class_PID Class_PID_Pitch_Angle_IMU;
    Class_PID Class_PID_Pitch_Omega_IMU;

    void Init();

    void TIM_100ms_Alive_PeriodElapedCallback();
    void TIM_1ms_Resolution_PeriodElapedCallback();
    void TIM_1ms_Control_PeriodElapedCallback();

    // inline float Get_Now_Yaw_Relative_Angle();
    inline float Get_Now_Yaw_Angle();
    inline float Get_Now_Pitch_Angle();
    inline float Get_Now_Yaw_Omega();
    inline float Get_Now_Pitch_Omega();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();

    inline void Set_Chassis_Yaw_Omega(float __Chassis_Yaw_Omega);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);

    // 返回归一化后的云台相对底盘角，范围 [-PI, PI]
    inline float Get_Now_Yaw_Relative_Angle();

    // 调试用：原始编码器角度 / 连续解算角度
    inline float Get_Now_Yaw_Relative_Raw_Angle();
    inline float Get_Now_Yaw_Relative_Continuous_Angle();

    inline float Get_Omega();

public:
    // 方向/零位
    float Yaw_Direction = 1.0f;
    float Pitch_Direction = -1.0f;

    float Yaw_Motor_Direction   = 1.0f;
    float Pitch_Motor_Direction = 1.0f;

    float Yaw_Offset_Rad   = 0.0f;
    float Pitch_Offset_Rad = 0.0f;

    // Pitch 限位（rad）
    float Min_Pitch_Angle = -0.15f;
    float Max_Pitch_Angle = 0.66f;

    //

    // 力矩限幅（Nm）
    float Yaw_Torque_Limit   = 8.0f;
    float Pitch_Torque_Limit = 9.0f;


protected:
    // yaw轴阻力前馈()
    float Yaw_Feed_Torque = 0.5f;
    // pitch轴重力补偿
    float Pitch_Gravity_Feed_Torque = 0.0f;

    float Now_Yaw_Angle = 0.0f;
    float Now_Pitch_Angle = 0.0f;
    float Now_Yaw_Omega = 0.0f;
    float Now_Pitch_Omega = 0.0f;
    float Now_Pitch_Omega_Raw = 0.0f;       //未滤波p轴角速度
    float Now_Yaw_Omega_Raw = 0.0f;         //未滤波y轴角度
    float Now_Yaw_Encoder_Angle = 0.0f;


    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    float Target_Yaw_Angle = 0.0f;
    float Target_Pitch_Angle = 0.0f;

    // yaw 解缠
    bool  yaw_inited   = false;
    float yaw_deg_last = 0.0f;
    float yaw_rad_cont = 0.0f;

    // 上电状态
    Enum_Gimbal_Startup_State Startup_State = Gimbal_Startup_WAIT_IMU;
    uint16_t imu_ready_cnt_ms = 0;

    Class_Filter_Frequency<5> Filter_Pitch_Omega;
    Class_Filter_Frequency<5> Filter_Yaw_Omega;

protected:
    // ================= Yaw 加速度前馈 =================
    // 拟合值 J = 0.12814716
    float Yaw_Acc_FF_K = 0.128f;

    // 目标角速度求导得到目标角加速度
    float Yaw_Target_Omega_Last = 0.0f;
    float Yaw_Target_Alpha = 0.0f;
    float Yaw_Target_Alpha_LPF = 0.0f;

    // 目标角加速度限幅，防止遥控输入突变导致前馈冲击
    float Yaw_Target_Alpha_Limit = 30.0f;

    // 加速度前馈力矩限幅
    float Yaw_Acc_FF_Torque_Limit = 0.8f;

protected:
    // ==============================
    // 底盘角速度前馈补偿
    // ==============================
    // 底盘当前 yaw 角速度，单位 rad/s
    float Chassis_Yaw_Omega = 0.0f;

    // 底盘角速度低通，防止轮速解算噪声直接进云台速度环
    float Chassis_Yaw_Omega_LPF = 0.0f;

    // 低通系数，越大响应越快，越小越平滑
    float Chassis_Yaw_Omega_LPF_Alpha = 0.25f;

    // 底盘角速度补偿系数
    float Chassis_Yaw_Omega_FF_K = 1.2f;


protected:

    // ==============================
    // Yaw 速度环手写 PI-D
    // ==============================
    float Yaw_Omega_Kp = 0.95f;
    float Yaw_Omega_Ki = 15.0f;
    float Yaw_Omega_Kd = 0.002f;

    // 积分输出限幅，不要再给到 5Nm
    float Yaw_Omega_I_Out_Max = 4.0f;

    // 速度环总输出限幅
    float Yaw_Omega_Out_Max = 7.0f;

    // 只有速度误差较小时才允许强积分
    float Yaw_Omega_I_Active_Error = 1.2f;

    // 目标速度较大时，积分弱化
    float Yaw_Omega_I_Target_Weak = 4.0f;

    // 输出接近饱和时不积分
    float Yaw_Omega_Saturation_Protect = 0.85f;

    // 积分泄放系数
    float Yaw_Omega_I_Leak = 0.995f;

    // 速度环积分输出，单位 Nm
    float Yaw_Omega_I_Torque = 0.0f;

    // 上一次速度误差
    float Yaw_Omega_Pre_Error = 0.0f;

    // 速度环分量，方便 VOFA 看
    float Yaw_Omega_P_Torque = 0.0f;
    float Yaw_Omega_D_Torque = 0.0f;
    float Yaw_Omega_PID_Torque = 0.0f;

    // ==============================
    // Yaw 位置到目标速度
    // ==============================
    float Yaw_Position_Kp = 4.2f;
    float Yaw_Position_Damping = 0.55f;

    float Yaw_Target_Omega_Max = 4.0f;
    float Yaw_Target_Omega_Cmd = 0.0f;

    float Yaw_Angle_Stop_Rad = 0.006f;
    float Yaw_Omega_Stop_Radps = 0.04f;

    // ==============================
    // Yaw 加速度前馈
    // ==============================
    bool Yaw_Acc_FF_Enable = true;


protected:
    float Normalize_Delta_Rad(float delta);
    float Yaw_Unwrap_To_Cont_Rad(float yaw_deg);
    float Yaw_Omega_Control_Calculate(float target_omega);

    void Self_Resolution();
    void Output();




public:
    // ================= Yaw惯量/摩擦辨识测试 =================
    bool Yaw_Inertia_Test_Enable = false;

    // 扫频测试力矩最大值，先用 0.5~0.8
    float Yaw_Inertia_Test_Torque_Max = 0.8f;

    // 扫频范围，单位 Hz
    float Yaw_Inertia_Test_Freq_Min = 0.15f;
    float Yaw_Inertia_Test_Freq_Max = 1.20f;

    // 一轮扫频时间，单位 s
    float Yaw_Inertia_Test_Sweep_Time = 20.0f;

    // 开始时力矩渐入，防止一下子冲击
    float Yaw_Inertia_Test_Ramp_Time = 2.0f;

    float Yaw_Inertia_Test_Time = 0.0f;
    float Yaw_Inertia_Test_Phase = 0.0f;
    float Yaw_Inertia_Test_Torque = 0.0f;
    float Yaw_Inertia_Test_Freq_Now = 0.0f;

    void Set_Yaw_Inertia_Test_Enable(bool enable);

    inline bool Get_Yaw_Inertia_Test_Enable()
    {
        return Yaw_Inertia_Test_Enable;
    }

    inline float Get_Yaw_Inertia_Test_Torque()
    {
        return Yaw_Inertia_Test_Torque;
    }

    inline float Get_Yaw_Inertia_Test_Freq()
    {
        return Yaw_Inertia_Test_Freq_Now;
    }

protected:
    void Yaw_Inertia_Sweep_Control();
    // ==============================
    // Yaw 电机编码器连续角度解算
    // ==============================
    // 达妙 MIT 反馈原始角度范围一般是 [-P_MAX, P_MAX]
    float Now_Yaw_Relative_Raw_Angle = 0.0f;

    // 连续解算后的角度，不直接给底盘 PID 使用
    float Now_Yaw_Relative_Continuous_Angle = 0.0f;

    // 对连续角度按 2PI 归一化后的相对角，给底盘跟随和坐标变换使用
    float Now_Yaw_Relative_Angle = 0.0f;

    float Yaw_Relative_Raw_Last = 0.0f;
    bool Yaw_Relative_Encoder_Inited = false;

    void Update_Yaw_Relative_Encoder_Angle();
};



inline float Class_Gimbal::Get_Now_Yaw_Relative_Angle()
{
    // 给底盘跟随和坐标变换使用：始终限制在 [-PI, PI]
    return Now_Yaw_Relative_Angle;
}

inline float Class_Gimbal::Get_Now_Yaw_Relative_Raw_Angle()
{
    return Now_Yaw_Relative_Raw_Angle;
}

inline float Class_Gimbal::Get_Now_Yaw_Relative_Continuous_Angle()
{
    return Now_Yaw_Relative_Continuous_Angle;
}
// ---------------- inline 实现 ----------------
inline void Class_Gimbal::Set_Chassis_Yaw_Omega(float __Chassis_Yaw_Omega)
{
    Chassis_Yaw_Omega_LPF += Chassis_Yaw_Omega_LPF_Alpha *
                             (__Chassis_Yaw_Omega - Chassis_Yaw_Omega_LPF);

    Chassis_Yaw_Omega = Chassis_Yaw_Omega_LPF;
}
inline float Class_Gimbal::Get_Now_Yaw_Angle()   { return (Now_Yaw_Angle); }
inline float Class_Gimbal::Get_Now_Pitch_Angle() { return Now_Pitch_Angle; }
inline float Class_Gimbal::Get_Now_Yaw_Omega()   { return Now_Yaw_Omega; }
inline float Class_Gimbal::Get_Now_Pitch_Omega() { return Now_Pitch_Omega; }

inline float Class_Gimbal::Get_Target_Yaw_Angle()   { return Target_Yaw_Angle; }
inline float Class_Gimbal::Get_Target_Pitch_Angle() { return Target_Pitch_Angle; }

inline void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)     { Target_Yaw_Angle = __Target_Yaw_Angle; }
inline void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle) { Target_Pitch_Angle = __Target_Pitch_Angle; }

inline void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    // 从 DISABLE -> 使能：锁住当前角为目标，避免一上电抽一下
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE && __Gimbal_Control_Type != Gimbal_Control_Type_DISABLE)
    {
    }
    Gimbal_Control_Type = __Gimbal_Control_Type;
}
inline float Class_Gimbal::Get_Omega()
{
    return Now_Yaw_Omega_Raw;
}


#endif //TEST_ROBOWAKER_GIMBAL_CONTROL_H