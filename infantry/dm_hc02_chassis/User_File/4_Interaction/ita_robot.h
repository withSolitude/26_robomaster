//
// Created by Lenovo on 2026/1/12.
//

#ifndef TEST_ROBOWAKER_ITA_ROBOT_H
#define TEST_ROBOWAKER_ITA_ROBOT_H

#include "3_Robot/Chassis/Chassis_Control.h"
#include "2_Device/Image_Transform/dvc_image_transform.h"
#include "1_Middleware/2_Algorithm/Slope/alg_slope.h"
#include "3_Robot/Slave_Connection_With_Master/Slave_With_Master.h"
#include "2_Device/Referee/dvc_referee.h"
#include "1_Middleware/2_Algorithm/Timer/alg_timer.h"


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

    // 裁判系统
    Class_Referee referee;

    // 上板与下板通信
    Class_Slave_With_Master Slave_With_Master;

    // 底盘跟随PID

    //裁判系统对底盘功率控制PID

    // 遥控器
    Class_Image_Transform VT13;

    // 裁判系统

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

    // 发射

    void Init();

    void Loop();

    void TIM_1000ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Calculate_Callback();

    void TIM_10ms_Calculate_PeriodElapsedCallback();

    void TIM_2ms_Calculate_PeriodElapsedCallback();

    void TIM_1ms_Calculate_Callback();

protected:

    // 初始话相关常量

    // 常量

    // 遥控器拨轮换算到电机角度映射
    const float dial_to_angle =  1.2f;//0.7 ;

    // 遥控器遥感死区
    const float VT13_Dead_Zone = 0.03f;
    // 遥控器遥感对云台YAW灵敏度系数

    // 底盘解算方向对底盘速度的前馈
    const float Chassis_Omega_FeedForward = 0.00f;

    // 内部变量



    // 小陀螺模式是否使能
    Enum_Robot_Gyroscope_Type Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;

    // 底盘跟随模式是否使能
    bool Chassis_Follow_Mode_Status = false;


    // 读变量

    // 写变量

    // 读写变量

    // 内部函数


    uint8_t Referee_IS_Valuiable();

    float Power_Calculate();

    void _Status_Control();

    void _Chassis_Control();


};



#endif //TEST_ROBOWAKER_ITA_ROBOT_H
