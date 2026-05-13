// //
// // Created by Lenovo on 2026/1/13.
// //
//
// #ifndef TEST_ROBOWAKER_GIMBAL_CONTROL_H
// #define TEST_ROBOWAKER_GIMBAL_CONTROL_H
//
// #include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
//
// /**
//  *@brief
//  *
//  */
// enum Enum_Gimbal_Control_Type
// {
//     Gimbal_Control_Type_DISABLE = 0,
//     Gimbal_Control_Type_POSITION,
// };
//
// /**
//  *@brief
//  *
//  */
// class Class_Gimbal
// {
// public:
//
//     // 云台陀螺仪
//
//     //  底盘陀螺仪
//
//     // yaw轴电机
//     Class_Gimbal_Yaw_Motor_DM_4310 Motot_Yaw;
//     // pitch轴电机
//     Class_Gimbal_Pitch_Motor_DM_4310 Motor_Pitch;
//
//     void Init();
//
//     inline float Get_Now_Yaw_Angle();
//
//     inline float Get_Now_Pitch_Angle();
//
//     inline float Get_Now_Yaw_Omega();
//
//     inline float Get_Now_Pitch_Omega();
//
//     inline float
//
// };
//
//
//
// #endif //TEST_ROBOWAKER_GIMBAL_CONTROL_H