// //
// // Created by Lenovo on 2026/1/17.
// //
//
// #ifndef TEST_ROBOWAKER_SHOOT_CONTROL_H
// #define TEST_ROBOWAKER_SHOOT_CONTROL_H
//
// #include "Shoot_Motor_Control.h"
// #include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
// #include "Motor/Motor_DM/dvc_motor_dm.h"
// #inlcude "Shoot_Motor_Control.h"
//
// // 大疆电机状态
// enum Enum_Shoot_DJI_Motor_Status
// {
//     Shoot_DJI_Motor_Status_DISABLE = 0,
//     Shoot_DJI_Motor_Status_NORMAL,
// };
//
// // 达妙电机状态
// enum Enum_Shoot_DM_Motor_Status
// {
//     Shoot_DM_Motor_Status_DISABLE = 0,
//     Shoot_DM_MOTOR_Status_ENABLE,
// };
//
// // 发射机构模式
// enum Enum_Shoot_Control_Mode
// {
//     Shoot_Mode_DISABLE = 0,
//     Shoot_Mode_FIRE,
//     Shoot_Mode_STOP,
//     Shoot_Mode_AUTO,
// };
//
//
//
// // 发射机构类
// class Class_Shoot
// {
// public:
//
//     // 摩擦轮电机
//     Class_Friction_Motor_DJI_C620 Class_Friction_Left_Front;
//     Class_Friction_Motor_DJI_C620 Class_Friction_Right_Front;
//     Class_Friction_Motor_DJI_C620 Class_Friction_Left_Behind;
//     Class_Friction_Motor_DJI_C620 Class_Friction_Right_Behind;
//     // 拨弹盘电机
//     Class_Shoot_Motor_DM_Normal Class_Shoot__Motor;
//
//     void Init();
//
//
//
// protected:
//
//     // 初始化相关常量
//
//     // 常量
//
//     // 拨弹盘一圈的子弹数
//     uint8_t Shoot_Round_Numbers = 8;
//
//     // 热量阈值
//
//     // 卡弹阈值
//
//
//
//     // 内部变量
//
//     // 读变量
//
//     // 写变量
//
//     // 当前热量
//
//     //
//
//     // 读写变量
//
//     // 发射机构状态
//     Enum_Shoot_Control_Mode Shoot_Control_Mode = Shoot_Mode_FIRE;
//
//     // 摩擦轮角速度
//     float Friction_Omega = 700.0f;
//     // 拨弹盘角速度
//     float BOOT_Omega = 1.0f;
//
//     // 内部函数
// };
//
// /**
//  *@brief 获取发射机构状态
//  *
//  */
// inline Eunm_Shoot_Control_Mode Class_Shoot::Get_Now_Shoot_Control_Mode()
// {
//     return (Shoot_Control_Mode);
// }
//
// /**
//  *@brief 获取摩擦轮角速度
//  *
//  */
// inline float Class_Shoot::Get_Friction_Omega()
// {
//     return (Friction_Omega);
// }
//
// /**
//  *@brief 获取拨弹电机角速度
//  *
//  */
// inline float Class_Shoot::Get_Boot_Omega()
// {
//     return (BOOT_Omega);
// }
//
//
//
// #endif //TEST_ROBOWAKER_SHOOT_CONTROL_H