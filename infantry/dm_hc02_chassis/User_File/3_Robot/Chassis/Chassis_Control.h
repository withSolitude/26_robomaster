//
// Created by Lenovo on 2025/11/19.
//

#ifndef TEST_ROBOWAKER_CHASSIS_CONTROL_H
#define TEST_ROBOWAKER_CHASSIS_CONTROL_H

#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Referee/dvc_referee.h"
//#include "2_Device/IMU/DM_IMU/dvc_dm_imu.h"
#include "1_Middleware/2_Algorithm/Slope/alg_slope.h"
#include "1_Middleware/2_Algorithm/RLS/alg_rls.h"



//底盘状态
enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_Wheel,
    Chassis_Control_Type_Wheel_Track_Leg,
};


//麦轮底盘
class Class_Chassis
{
public:

    //底盘速度xPID
    Class_PID PID_Vx;
    //底盘速度yPID
    Class_PID PID_Vy;
    //底盘角速度PID
    Class_PID PID_Omega;
    //底盘imu_pitch的PID
    Class_PID PID_Chassis_Pitch_Angle;

    //履带电机
    Class_Motor_DJI_C620  Track_Wheel[2];
    //腿达妙4340
    Class_Motor_DM_Normal Leg_Wheel[2];
    //底盘电机
    Class_Motor_DJI_C620 Motor_Wheel[4];

    void Init();

    inline void Set_Power_Limit_Max(float __Power_Limit_Max);

    inline float Get_Estimated_Power_Cmd();

    inline float Get_Estimated_Power_Limited();

    inline float Get_Now_Velocity_x();

    inline float Get_Now_Velocity_y();

    inline float Get_Now_Omega();

    inline float Get_Slope_Direction_X();

    inline float Get_Slope_Direction_Y();

    inline float Get_Slope_Direction_Z();

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();

    inline float Get_Target_Velocity_X();

    inline float Get_Target_Velocity_Y();

    inline float Get_Target_Omega();

    inline float Get_Now_Left_Track_Wheel_Omega();

    inline float Get_Now_Right_Track_Wheel_Omega();

    inline float Get_Now_Chassis_Pitch_Angle();

    inline float Get_Now_Left_Leg_Angle();

    inline float Get_Now_Right_Leg_Angle();

    inline float Get_Target_Chassis_Pitch_Angle();

    inline void Set_Chassis_Control_Typer(Enum_Chassis_Control_Type __Chassis_Control_Type);

    inline void Set_Target_Velocity_X(float __Target_Velocity_x);

    inline void Set_Target_Velocity_Y(float __Target_Velocity_y);

    inline void Set_Target_Omega(float __Target_Omega);

    inline void Set_Track_Motor_Omega(float __Target_Track_Wheel_Torque);

    inline void Set_Leg_Motor_Angle(float __Target_Leg_Angle);

    inline void Set_Target_Chassis_Pitch_Angle(float __Target_Chassis_Pitch_Angle);

    inline float Get_Target_Chassis_Leg_Motor_Angle();

    inline float Get_Target_Leg_Motor_Torque();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_2ms_Resolution_PeriodElapsedCallback();

    void TIM_2ms_Control_PeriodElapsedCallback();



    // S档功率优先：前两个麦轮不分配功率，功率只分给后两个麦轮和两个履带
    inline void Set_Track_Power_Priority_Status(bool __Status);

    inline bool Get_Track_Power_Priority_Status();


    inline float Get_Target_Cunrrent();
    inline float Get_now_omega_motor_raw();

protected:
    //初始化相关常量

    //底盘电机方向
    const int8_t Wheel_Motor_Direction[4] = {1,-1,-1,1};

    //履带电机方向
    const int8_t Track_Motor_Direction[2] = {-1,1};

    //腿电机方向
    const int8_t Leg_Motor_Direction[2] = {1,-1};

    //3508效率
    const float Geal_eff = 0.70f;

    //3508减速比
    const float GEAL_RATIO = 19.2f;

    //3508扭矩电流  (N/M)
    const float KT = 3.0;

    //麦轮半径
    const float Wheel_Radius = 0.1524977f;

    //轮轴距
    const float MECANUM_L =  (0.46+0.37)/2;//0.92f/2.0f;       //Lx+Ly(前后轮半轴距+左右轮半轴距)     //前后轴495，左右轴425

    //腿电机位置限幅
    const float Leg_Motor_Angle_Min = 0.0f;
    const float Leg_Motor_Angle_Max = 1.2f;

    //履带转矩限制
    const float Track_Wheel_Current_Limit = 40.0f;

    //腿电机角速度限制
    const float Leg_Motor_Omega_Limit = 24.0f;

    //腿电机力矩限制
    const float Leg_Motor_Torque_Limit = 27.0f;

    //内部变量
    //轮电机目标角速度
    float Target_Wheel_Omega[4];

    //轮电机目标电流
    float Target_Wheel_Current[4];

    //前馈
    float Target_Wheel_FF_Current[4];

    //轮速目标
    float Target_Wheel_Omega_Current[4];

    //轮电机静摩擦阻力电流值
    float Static_Resistance_Wheel_Current[4];

    //轮电机动摩擦阻力电流值
    float Dynamic_Resistance_Wheel_Current[4];

    //39->65
    // 最大功率
    float Chassis_Power_Limit_Max = 47.0f;
    // 估算功率指令
    float Chassis_Power_Est_Cmd = 0.0f;
    // 估算最大功率
    float Chassis_Power_Est_Limited = 0.0f;

    // 功率分配阈值(用于误差权重/功率权重/平滑切换)
    // float Power_Distribute_Error_Set = 20.0f;       //rad/s
    // float Power_Distribute_Prop_Set = 15.0f;        //rad/s
    float Power_Distribute_Error_Set = 35.0f;       //rad/s
    float Power_Distribute_Prop_Set = 15.0f;        //rad/s


    // // 功率限制后的电流平滑状态
    // float Power_Limit_Current_Last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    //
    // // 每个控制周期(2ms)允许的最大上升/下降电流变化量，单位 A/周期
    // float Power_Limit_Current_Rise_Step = 0.10f;
    // float Power_Limit_Current_Fall_Step = 0.45f;
    //
    // // 可选：限功电流一阶低通系数，0~1，越大越跟手
    // float Power_Limit_Current_Lpf_Alpha = 0.45f;


    // 模型：P = K0*I*ω + K1*ω^2 + K2*I^2 + A
    Class_RLS RLS;
    uint8_t Power_RLS_ENABLE = 0;
    float Power_K1_Est = 0.0f;
    float Power_K2_Est = 0.0f;

    //现在速度X
    float Now_Vx = 0.0f;
    //现在速度Y
    float Now_Vy  = 0.0f;
    //现在角速度
    float Now_Omega = 0.0f;
    //现在左履带的角速度
    float Now_Left_Track_Wheel_Omega = 0.0f;
    //现在右履带的角速度
    float Now_Right_Track_Wheel_Omega = 0.0f;
    //现在底盘pitch角度
    float Now_Chassis_Pitch_Angle = 0.0f;

    //现在腿角度
    float Now_Leg_Angle[2] = {0.0f,0.0f};
    //现在腿角速度
    float Now_Leg_Omega[2] = {0.0f,0.0f};
    //现在腿力矩
    float Now_Leg_Torque[2] = {0.0f,0.0f};
    //现在履带角速度
    float Now_Track_Wheel_Omega[2] = {0.0f,0.0f};
    //腿力矩前馈
    float Target_Leg_Motor_FF[2] = {0.0f,0.0f};



    // 斜坡法向量在底盘方向向量
    float Slope_Direction_X = 0.0f;
    float Slope_Direction_Y = 0.0f;
    float Slope_Direction_Z = 1.0f;

    //读写变量

    //底盘控制方法
    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;

    //目标速度X
    float Target_Vx = 0.0f;
    //目标速度Y
    float Target_Vy = 0.0f;
    //目标角速度
    float Target_Omega = 0.0f;
    //履带目标角速度
    float Target_Track_Wheel_Omega = 5.0f;
    //履带目标力矩
    float Target_Track_Wheel_Torque[2] = {0.0f};
    //底盘pitch目标角度
    float Target_Chassis_Pitch_Angle = 0.0f;
    //腿目标角度
    float Target_Leg_Angle = 0.0f;
    //腿目标角速度
    float Target_Leg_Omega[2] = {0.0f};
    //腿目标力矩
    float Target_Leg_Torque[2] = {0.0f};


    // S档功率优先模式
    // false：正常 4麦轮 + 2履带 功率分配
    // true ：前两个麦轮 Motor_Wheel[0]/[1] 不分配功率，后两个麦轮 + 两个履带分配功率
    bool Track_Power_Priority_Status = false;



    //内部函数

    void Self_Resolution();

    void Kinematics_Inverse_Resolution();

    //void _Steer_Motor_Kinematics_Nearest_Transposition();

    void Output_To_Dynamics();

    void Dynamics_Inverse_Resolution();

    void Output_To_Motor();

    void Leg_Control(const bool&Enable);

    void Leg_Motor_Output(const bool&Enable);

    void Power_Limit_Control();
};

/**
 * @brief 读取x轴速度
 *
 */
inline float Class_Chassis::Get_Now_Velocity_x()
{
    return Now_Vx;
}

/**
 *@brief 读取y轴速度
 *
 */
inline float Class_Chassis::Get_Now_Velocity_y()
{
    return Now_Vy;
}

/**
 *@brief 读取角速度
 *
 */
inline float Class_Chassis::Get_Now_Omega()
{
    return Now_Omega;
}

/**
 *@brief 读取左履带电机角速度
 *
 */
inline float Class_Chassis::Get_Now_Left_Track_Wheel_Omega()
{
    return Now_Track_Wheel_Omega[0];
}

/**
 *@brief 读取右履带电机角速度
 *
 */
inline float Class_Chassis::Get_Now_Right_Track_Wheel_Omega()
{
    return Now_Track_Wheel_Omega[1];
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_X()
{
    return (Slope_Direction_X);
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_Y()
{
    return (Slope_Direction_Y);
}

/**
 * @brief 获取斜坡法向量在底盘方向向量
 *
 * @return float 斜坡法向量在底盘方向向量
 */
inline float Class_Chassis::Get_Slope_Direction_Z()
{
    return (Slope_Direction_Z);
}

/**
 * @brief 获取底盘控制方法
 *
 * @return Enum_Chassis_Control_Type 底盘控制方法
 */
inline Enum_Chassis_Control_Type Class_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

/**
 * @brief 获取目标速度X
 *
 * @return float 目标速度X
 */
inline float Class_Chassis::Get_Target_Velocity_X()
{
    return (Target_Vx);
}

/**
 * @brief 获取目标速度Y
 *
 * @return float 目标速度Y
 */
inline float Class_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Vy);
}

/**
 * @brief 获取目标速度方向
 *
 * @return float 目标速度方向
 */
inline float Class_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

/**
 * @brief 设定底盘控制方法
 *
 * @param __Chassis_Control_Type 底盘控制方法
 */
inline void Class_Chassis::Set_Chassis_Control_Typer(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

/**
 * @brief 设定目标速度X
 *
 * @param __Target_Velocity_X 目标速度X
 */
inline void Class_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Vx = __Target_Velocity_X;
}

/**
 * @brief 设定目标速度Y
 *
 * @param __Target_Velocity_Y 目标速度Y
 */
inline void Class_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Vy = __Target_Velocity_Y;
}

/**
 * @brief 设定目标角速度
 *
 * @param __Target_Omega 目标角速度
 */
inline void Class_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

/**
 *@brief 设定履带转速
 *
 */
inline void Class_Chassis::Set_Track_Motor_Omega(float __Target_Track_Wheel_Omega)
{
    Target_Track_Wheel_Omega = __Target_Track_Wheel_Omega;
}

/**
 *@brief 设定腿电机角度
 *
 */
inline void Class_Chassis::Set_Leg_Motor_Angle(float __Target_Leg_Angle)
{
    Target_Leg_Angle = __Target_Leg_Angle;
}

inline void Class_Chassis::Set_Power_Limit_Max(float __Power_Limit_Max)
{
    Chassis_Power_Limit_Max = __Power_Limit_Max;
}

inline float Class_Chassis::Get_Estimated_Power_Cmd()
{
    return Chassis_Power_Est_Cmd;
}

inline float Class_Chassis::Get_Estimated_Power_Limited()
{
    return Chassis_Power_Est_Limited;
}

inline float Class_Chassis::Get_Now_Chassis_Pitch_Angle()
{
    return Now_Chassis_Pitch_Angle;
}


inline float Class_Chassis::Get_Target_Chassis_Pitch_Angle()
{
    return Target_Chassis_Pitch_Angle;
}

inline void Class_Chassis::Set_Target_Chassis_Pitch_Angle(float __Target_Chassis_Pitch_Angle)
{
    Target_Chassis_Pitch_Angle = __Target_Chassis_Pitch_Angle;
}

inline float Class_Chassis::Get_Target_Chassis_Leg_Motor_Angle()
{
    return Target_Leg_Angle;
}



// 调试用
inline float Class_Chassis::Get_Target_Leg_Motor_Torque()
{
    return Target_Leg_Torque[0];
}

inline float Class_Chassis::Get_Target_Cunrrent()
{
    return Target_Wheel_Current[0];
}
inline float Class_Chassis::Get_now_omega_motor_raw()
{
    return Motor_Wheel[0].Get_Now_Omega()*GEAL_RATIO;
}





inline void Class_Chassis::Set_Track_Power_Priority_Status(bool __Status)
{
    Track_Power_Priority_Status = __Status;
}

inline bool Class_Chassis::Get_Track_Power_Priority_Status()
{
    return Track_Power_Priority_Status;
}

#endif //TEST_ROBOWAKER_CHASSIS_CONTROL_H
