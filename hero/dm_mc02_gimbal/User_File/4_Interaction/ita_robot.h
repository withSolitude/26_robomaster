//
// Created by Lenovo on 2026/1/12.
//

#ifndef TEST_ROBOWAKER_ITA_ROBOT_H
#define TEST_ROBOWAKER_ITA_ROBOT_H

#include "3_Robot/Chassis/Chassis_Control.h"
#include "2_Device/Image_Transform/dvc_image_transform.h"
#include "1_Middleware/2_Algorithm/Slope/alg_slope.h"
#include "3_Robot/Gimbal/Gimbal_Control.h"
#include "3_Robot/Shoot/Shoot_Control.h"
#include "3_Robot/Slave_Connection_With_Master/Slave_With_Master.h"
#include "2_Device/Aimbot/dvc_aimbot.h"
#include "1_Middleware/2_Algorithm/Timer/alg_timer.h"


// 发射机构类型
enum Enum_Robot_Shoot_Type
{
    Robot_Shoot_Type_BURST = 0,
    Robot_Shoot_Type_CD,
};

/**
 *@brief 小陀螺类型
 *
 */
enum Enum_Robot_Gyroscope_Type
{
    Robot_Gyroscope_Type_DISABLE = 0,
    Robot_Gyroscope_Type_CLOCKWISE,
    Robot_Gyroscope_Type_ANTICLOCKWISE,
};

class Class_Robot;

class Class_Robot
{
public:

    // 自瞄
    Class_AutoAim AutoAim;

    // 裁判系统
    Class_Referee referee;

    // 底盘跟随PID
    Class_PID PID_Chassis_Follow;

    // 上下板通信
    Class_Slave_With_Master Slave_With_Master;

    // 底盘跟随PID

    //裁判系统对底盘功率控制PID

    // 遥控器
    Class_Image_Transform VT13;

    // 斜坡函数底盘速度x
    Class_Slope Slope_Speed_X;

    // 斜坡函数底盘速度y
    Class_Slope Slope_Speed_Y;

    // 斜坡函数底盘角速度
    Class_Slope Slope_Speed_Omega;

    // 斜坡函数履带角速度
    Class_Slope Slope_Track_Omega;

    // 斜坡函数腿电机角度
    Class_Slope Slope_Leg_Left_Angle;
    Class_Slope Slope_Leg_Right_Angle;

    // 底盘
    Class_Chassis Chassis;
    // 云台
    Class_Gimbal Gimbal;
    // 发射
    Class_Shoot Shooter;


    void Init();

    void Loop();

    void TIM_1000ms_Alive_PeriodElapsedCallback();

    void TIM_200ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Calculate_Callback();

    void TIM_10ms_Calculate_PeriodElapsedCallback();

    void TIM_2ms_Calculate_PeriodElapsedCallback();

    void TIM_1ms_Calculate_Callback();

protected:

    // 初始话相关常量


    // 遥控器拨轮换算到电机角度映射
    const float dial_to_angle = 0.5f;//0.7 ;

    // 遥控器遥感死区
    const float VT13_Dead_Zone = 0.03f;
    // 遥控器遥感对云台YAW灵敏度系数

    // 鼠标灵敏度
    float VT13_Mouse_Yaw_Angle_Resolution   = 0.6f;
    float VT13_Mouse_Pitch_Angle_Resolution = 0.5f;

    // 鼠标死区，防止静止时轻微漂
    const float VT13_Mouse_Dead_Zone = 0.00002f;

    // 内部变量

    // 发射机构属性
    Enum_Robot_Shoot_Type Robot_Shoot_Type = Robot_Shoot_Type_CD;

    // 鼠标左键长按判定计数
    uint16_t Shoot_Left_Mouse_Hold_Count = 0;
    // 长按阈值，单位 ms
    uint16_t Shoot_Left_Mouse_Hold_Threshold = 120;

    // 小陀螺模式是否使能
    Enum_Robot_Gyroscope_Type Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;

    // 底盘跟随模式是否使能
    bool Chassis_Follow_Mode_Status = true;
    // 底盘跟随总开关
    bool Chasssis_Follow_Status = true;

    // 底盘跟随死区
    // float Chassis_Follow_Dead_Zone = 0.05;

    // 底盘解算方向对底盘速度的前馈
    const float Chassis_Omega_FeedForward = 0.08f;

    // 云台相对底盘零位
    float Gimbal_Yaw_Relative_Zero_Rad = 0.0f;


    // 自瞄
    // 自瞄开关：鼠标右键按住开启
    bool AutoAim_Enable = false;

    // 是否允许视觉 mode=2 触发拨弹
    bool AutoAim_Auto_Fire_Enable = false;

    // 默认弹速，单位按上位机约定，通常 m/s
    float AutoAim_Bullet_Speed = 12.0f;

    // 发弹计数
    uint16_t AutoAim_Bullet_Count = 0;

    uint8_t Referee_IS_Valuiable();

    void _Chassis_Gyro_Start();
    void _Chassis_Gyro_Stop(float yaw_relative);

    float Power_Calculate();

    void _AutoAim_Control();

    void _Status_Control();

    void _Chassis_Control();

    void _Gimbal_Control();

    void _Shooter_Control();

    // 摩擦轮目标速度
    float Target_Friction_Omega  = 22.7f;

    // 摩擦轮开关
    bool Friction_Enable = false;
    // 开火间隔时间

    // 转头标志位
    bool Gimbal_Behind_Flag = false;

};



#endif //TEST_ROBOWAKER_ITA_ROBOT_H
