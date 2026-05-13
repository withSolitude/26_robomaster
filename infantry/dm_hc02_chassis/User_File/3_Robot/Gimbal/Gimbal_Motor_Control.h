// //
// // Created by Lenovo on 2026/1/14.
// //
//
// #ifndef TEST_ROBOWAKER_GIMBAL_MOTOR_CONTROL_H
// #define TEST_ROBOWAKER_GIMBAL_MOTOR_CONTROL_H
//
// #include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
//
//
// /**
//  *@brief yaw轴电机
//  *
//  */
// class Class_Gimbal_Yaw_Motor_DM_4310
// {
// public:
//
//     // 云台陀螺仪
//
//     // 底盘陀螺仪
//
//
//     // 陀螺仪角度环PID
//
//     // 陀螺仪角速度环PID
//
//     inline float Get_Now_AHRS_Omega();
//
//     void TIM_100ms_Alive_PeriodElapsedCallback();
//
//     void TIM_Calculate_PeriodElapsedCallback();
//
// protected:
//
//     // 初始化相关常量
//
//     // 常量
//
//     // 内部变量
//
//     // 读变量
//
//     // 写变量
//
//     // 读写变量
//
//     // 内部变量
//
//     void PID_Calculate();
// };
//
// /**
//  *@brief pitch轴电机
//  *
//  */
// class Class_Gimbal_Pitch_Motor_DM_4310
// {
// public:
//
//     // 云台陀螺仪
//
//     // 陀螺仪角度环PID
//
//     // 陀螺仪角速度环PID
//
//     inline float Get_Now_AHRS_Omega();
//
//     void TIM_100ms_Alive_PeriodElapsedCallback();
//
//     void TIM_Calculate_PeriodElapsedCallback();
//
// protected:
//     // 初始化相关常量
//
//     // 常量
//
//     // 内部变量
//
//     // 读变量
//
//     // 写变量
//
//     // 读写变量
//
//     // 内部变量
//
//     void PID_Calculate();
// };
//
//
//
//
// #endif //TEST_ROBOWAKER_GIMBAL_MOTOR_CONTROL_H