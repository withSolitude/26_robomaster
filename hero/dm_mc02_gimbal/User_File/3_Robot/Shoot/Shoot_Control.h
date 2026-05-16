#ifndef TEST_ROBOWAKER_SHOOT_CONTROL_H
#define TEST_ROBOWAKER_SHOOT_CONTROL_H

#include "Shoot_Motor_Control.h"
#include "1_Middleware/2_Algorithm/FSM/alg_fsm.h"

/**
 * @brief 发射机构控制模式
 *
 */
enum Enum_Shoot_Control_Type
{
    Shoot_Control_Type_DISABLE = 0,
    Shoot_Control_Type_CEASEFIRE,
    Shoot_Control_Type_SPOT,
    Shoot_Control_Type_AUTO,
};

/**
 * @brief 防卡弹状态类型
 *
 */
enum Enum_FSM_Shoot_Anti_Jamming_Status
{
    FSM_Shoot_Anti_Jamming_Status_NORMAL = 0,
    FSM_Shoot_Anti_Jamming_Status_JAMMING_SUSPECT,
    FSM_Shoot_Anti_Jamming_Status_JAMMING_CONFIRM,
    FSM_Shoot_Anti_Jamming_Status_PROCESSING,
};

class Class_Shoot;

/**
 * @brief 拨弹防卡弹有限状态机
 *
 */
class Class_FSM_Shoot_Anti_Jamming : public Class_FSM<4>
{
public:
    Class_Shoot *Shoot = nullptr;

    void TIM_Calculate_PeriodElapsedCallback();

protected:
    // 扭矩过大判定阈值，按4340实际调
    float Driver_Torque_Threshold = 4.0f;
    // 速度很低时更像卡弹
    float Driver_Omega_Threshold = 0.1f;
    // 卡弹嫌疑持续时间
    uint32_t Jamming_Suspect_Time_Threshold = 60;
    // 回拨处理持续时间
    uint32_t Jamming_Solving_Time_Threshold = 120;
    // 回拨角度
    float Driver_Back_Angle = PI / 6.0f;
};

/**
 * @brief 发射机构
 *
 */
class Class_Shoot
{
public:
    friend class Class_FSM_Shoot_Anti_Jamming;

    Class_FSM_Shoot_Anti_Jamming FSM_Anti_Jamming;

    // 三摩擦轮3508
    Class_Shoot_Friction_Motor_DJI_C620 Motor_Friction_1;
    Class_Shoot_Friction_Motor_DJI_C620 Motor_Friction_2;
    Class_Shoot_Friction_Motor_DJI_C620 Motor_Friction_3;

    // 拨弹4340
    Class_Shoot_Driver_Motor_DM_Normal Motor_Driver;

    void Init();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_1ms_Calculate_PeriodElapsedCallback();

    inline Enum_Shoot_Control_Type Get_Shoot_Control_Type() const
    {
        return Shoot_Control_Type;
    }

    inline float Get_Friction_Omega() const
    {
        return Friction_Omega;
    }

    inline float Get_Target_Ammo_Shoot_Frequency() const
    {
        return Target_Ammo_Shoot_Frequency;
    }

    inline float Get_Now_Ammo_Shoot_Frequency() const
    {
        return Now_Ammo_Shoot_Frequency;
    }

    inline void Set_Shoot_Control_Type(const Enum_Shoot_Control_Type __type)
    {
        // 从发射状态切回停火时，锁住当前位置，避免拨弹电机突然回零
        if (__type == Shoot_Control_Type_CEASEFIRE &&
            Shoot_Control_Type != Shoot_Control_Type_CEASEFIRE)
        {
            // Driver_Target_Angle = Motor_Driver.Get_Now_Angle();
            Driver_Target_Angle = Motor_Driver.Get_Now_Angle_Continuous();
        }

        Shoot_Control_Type = __type;
    }

    inline void Set_Friction_Omega(const float __omega)
    {
        Friction_Omega = __omega;
    }

    inline void Set_Target_Ammo_Shoot_Frequency(const float __freq)
    {
        Target_Ammo_Shoot_Frequency = __freq;
    }

    inline void Set_Now_Heat(const float __heat)
    {
        Now_Heat = __heat;
    }

    inline void Set_Heat_Limit_Max(const float __heat_limit_max)
    {
        Heat_Limit_Max = __heat_limit_max;
    }

    inline void Set_Heat_CD(const float __heat_cd)
    {
        Heat_CD = __heat_cd;
    }


    inline void Set_Driver_Debug_Enable(const bool __enable)
    {
        if (__enable && !Driver_Debug_Enable)
        {
            Driver_Debug_Target_Angle = Motor_Driver.Get_Now_Angle();

            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        }

        Driver_Debug_Enable = __enable;
    }


protected:
    // 一圈对应弹丸数
    float Ammo_Num_Per_Round = 6.0f;

    // 三个摩擦轮方向，按实际安装改
    float Friction_Direction[3] = {1.0f, 1.0f, 1.0f};

    // 拨弹方向，若反了改成 -1
    float Driver_Direction = 1.0f;

    // 单发拨弹角度
    float Driver_Angle_Per_Bullet = 2.0f * PI / 6.0f;

    // 单发完成判定阈值
    float Driver_Finish_Angle_Threshold = 0.08f;
    float Driver_Finish_Omega_Threshold = 0.0f;

    // 停火阈值和减速阈值
    float Heat_Limit_Ceasefire_Threshold = 0.0f;
    float Heat_Limit_Slowdown_Threshold = 0.0f;

    // 当前热量
    float Now_Heat = 0.0f;
    // 热量上限
    float Heat_Limit_Max = 0.0f;
    // 热量冷却
    float Heat_CD = 0.0f;
    // 每发消耗热量
    float Per_Heat = 100.0f;
    // 热量标志位
    bool Heat_Enable = true;

    // 当前发射模式
    Enum_Shoot_Control_Type Shoot_Control_Type = Shoot_Control_Type_CEASEFIRE;

    // 三摩擦轮目标转速
    float Friction_Omega = 0.0f;

    // 目标射频（发/秒）
    float Target_Ammo_Shoot_Frequency = 0.0f;
    // 实际当前射频（经热量限速后的值）
    float Now_Ammo_Shoot_Frequency = 0.0f;

    // 拨弹目标角度（单发/停火保持）
    float Driver_Target_Angle = 0.0f;

    // 内部函数
    void Heat_Calculate();

    void Output();


protected:

    // 拨弹调试旁路开关
    bool Driver_Debug_Enable = false;

    // 拨弹调试目标角度
    float Driver_Debug_Target_Angle = 0.0f;
};

#endif
