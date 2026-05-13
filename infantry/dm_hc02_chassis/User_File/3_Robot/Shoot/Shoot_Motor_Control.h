// //
// // Created by Lenovo on 2026/1/17.
// //
//
// #ifndef TEST_ROBOWAKER_SHOOT_MOTOR_CONTROL_H
// #define TEST_ROBOWAKER_SHOOT_MOTOR_CONTROL_H
//
// #include "Motor/Motor_DJI/dvc_motor_dji.h"
// #include "Motor/Motor_DM/dvc_motor_dm.h"
// #include "1_Middleware/2_Algorithm/Filter/alg_filter.h"
//
// class Class_Friction_Motor_DJI_C620 : public Class_Motor_DJI_C620
// {
// public:
//     Class_Filter_Fourier<5> Filter_Fourier_Omega;
//
//     inline float Get_Now_Friction_Filter_Omega();
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
//     // 内部函数
//     void PID_Calculate();
// }
//
// class Class_Shoot_Motor_DM_Normal : public Class_Motor_DM_Normal
// {
// public:
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
//     // 内部函数
//
//
// };
//
// inline float Class_Friction_Motor_DJI_C620::Get_Now_Friction_Filter_Omega()
// {
//     return (Filter_Fourier_Omega.Get_Out());
// }
//
//
// #endif //TEST_ROBOWAKER_SHOOT_MOTOR_CONTROL_H