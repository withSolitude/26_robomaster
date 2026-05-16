#ifndef SHOOT_MOTOR_CONTROL_H
#define SHOOT_MOTOR_CONTROL_H

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "1_Middleware/2_Algorithm/Filter/Frequency/alg_filter_frequency.h"
#include "1_Middleware/2_Algorithm/PID/alg_pid.h"
#include "1_Middleware/2_Algorithm/Basic/alg_basic.h"

/**
 * @brief 拨弹电机控制方式
 *
 */
enum Enum_Shoot_Driver_Control_Type
{
    Shoot_Driver_Control_Type_DISABLE = 0,
    Shoot_Driver_Control_Type_ANGLE,
    Shoot_Driver_Control_Type_OMEGA,
    Shoot_Driver_Control_Type_TORQUE,
};

/**
 * @brief 摩擦轮3508电机，带速度滤波
 *
 */
class Class_Shoot_Friction_Motor_DJI_C620 : public Class_Motor_DJI_C620
{
public:

    Class_Filter_Frequency<5> Filter_Fourier_Omega;

    inline float Get_Now_Filter_Omega() const
    {
        return Filter_Fourier_Omega.Get_Out();
    }

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    void PID_Calculate();
};

/**
 * @brief 拨弹4340，达妙MIT模式，外部角度/速度双环
 *
 */
class Class_Shoot_Driver_Motor_DM_Normal : public Class_Motor_DM_Normal
{
public:
    // 名字沿用旧版，便于 Shoot_Control.cpp 直接使用
    Class_Filter_Frequency<5> Filter_Fourier_Omega;
    Class_PID PID_Angle;
    Class_PID PID_Omega;

    inline Enum_Shoot_Driver_Control_Type Get_Shoot_Driver_Control_Type() const
    {
        return Shoot_Driver_Control_Type;
    }

    inline float Get_Target_Angle() const
    {
        return Target_Angle;
    }

    inline float Get_Target_Omega() const
    {
        return Target_Omega;
    }

    inline float Get_Target_Torque() const
    {
        return Target_Torque;
    }

    inline float Get_Now_Filter_Omega() const
    {
        return Filter_Fourier_Omega.Get_Out();
    }

    inline void Set_Shoot_Driver_Control_Type(const Enum_Shoot_Driver_Control_Type __type)
    {
        Shoot_Driver_Control_Type = __type;
    }

    inline void Set_Target_Angle(const float __target_angle)
    {
        Target_Angle = __target_angle;
    }

    inline void Set_Target_Omega(const float __target_omega)
    {
        Target_Omega = __target_omega;
    }

    inline void Set_Target_Torque(const float __target_torque)
    {
        Target_Torque = __target_torque;
    }

    inline void Set_Feedforward_Omega(const float __feedforward_omega)
    {
        Feedforward_Omega = __feedforward_omega;
    }

    inline void Set_Feedforward_Torque(const float __feedforward_torque)
    {
        Feedforward_Torque = __feedforward_torque;
    }


    inline float Get_Now_Angle_Continuous() const
    {
        return Now_Angle_Continuous;
    }

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    Enum_Shoot_Driver_Control_Type Shoot_Driver_Control_Type = Shoot_Driver_Control_Type_DISABLE;

    float Target_Angle = 0.0f;
    float Target_Omega = 0.0f;
    float Target_Torque = 0.0f;
    float Feedforward_Omega = 0.0f;
    float Feedforward_Torque = 0.0f;

    // 最终输出扭矩限幅，按机构再调
    float Torque_Limit = 7.5f;

    /**
 * @brief 拨弹电机角度展开变量
 */
    bool Angle_Unwrap_Init_Flag = false;
    float Pre_Angle_Wrap = 0.0f;
    int32_t Angle_Total_Round = 0;
    float Now_Angle_Continuous = 0.0f;

    void Update_Angle_Unwrap();

    void PID_Calculate();
};

#endif
