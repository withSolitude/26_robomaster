//
// Created by Lenovo on 2026/1/12.
//
#include "5_Task/tsk_config_and_callback.h"
#include "4_Interaction/ita_robot.h"
#include "2_Device/Vofa/dvc_vofa.h"
#include "3_Robot/Chassis/Chassis_Control.h"
#include "2_Device/Vofa/dvc_Vofa_Tuner.h"
#include "2_Device/BSP/Power/bsp_power.h"
#include "2_Device/BMI088/imu.h"
//调试专用
extern float q[4];
float a;
float b;
float c;
float d;
float e;
float f;
float g;
//
float tar_angle;
float now_angle;
float tar_omega;
float now_omega;
float tar_torque;
float now_torque;

// //
// float w_1;
// float w_2;
// float w_3;
//
float tar_w_1;
float tar_w_2;
float tar_w_3;
//
float w_1_filter;
float w_2_filter;
float w_3_filter;
//
// float w_1_current;
// float w_1_now_current;

//
float pitch;
float yaw;
float roll;

float P;
float Y;
float R;

// float vt;
// float target_angle;
// float target_omega;
// float target_torque;
// float now_angle;
// float now_omega;
// float now_torque;

// pid
float kp;
float ki;
float kd;
float kf;
float i_out_max;
float out_max;
float d_t;

// 遥控
float Vx;
float Vy;
float Omega;

float pid_vx_out;
// 底盘

// 实际速度
float measure_vx;
float measure_vy;
float measure_omega;

// 麦轮底盘目标转速
float Chassis_Motor_0_Tar_Omega;
float Chassis_Motor_1_Tar_Omega;
float Chassis_Motor_2_Tar_Omega;
float Chassis_Motor_3_Tar_Omega;
// 履带电机目标转速
float Track_Motor_0_Tar_Omega;
float Track_Motor_1_Tar_Omega;

// 麦轮底盘电机转速
float Chassis_Motor_0_Omega;
float Chassis_Motor_1_Omega;
float Chassis_Motor_2_Omega;
float Chassis_Motor_3_Omega;
// 履带3508电机转速
float Track_Motor_0_Omega;
float Track_Motor_1_Omega;

// 麦轮底盘电机电流
float Chassis_Motor_0_Current;
float Chassis_Motor_1_Current;
float Chassis_Motor_2_Current;
float Chassis_Motor_3_Current;
// 履带3508电机电流
float Track_Motor_0_Current;
float Track_Motor_1_Current;

// 底盘功率调试
float chassis_power_cmd_est = 0.0f;
float chassis_power_limited_est = 0.0f;
float chassis_power_limit_max_dbg = 0.0f;

// 云台

// 发射

bool volatile init_finished = false;
uint32_t flag = 0;

// 这就是我们的牢雄！！！！！
Class_Robot robot;
Class_Vofa_Tuner Vofa_Tuner;
Class_Vofa_JustFloat Vofa_JustFloat;

/**
 *@brief VOFA+ 串口绘图UART1初始化
 *
 */
void Vofa_JustFloat_Init(void)
{
    Vofa_JustFloat.Init(&huart7,8,20);
    //绑定通道
    // Vofa_JustFloat.Bind_Channel(0,&tar_angle);
    // Vofa_JustFloat.Bind_Channel(1,&now_angle);
    // Vofa_JustFloat.Bind_Channel(2,&tar_omega);
    // Vofa_JustFloat.Bind_Channel(3,&now_omega);
    // Vofa_JustFloat.Bind_Channel(4,&tar_torque);
    // Vofa_JustFloat.Bind_Channel(5,&now_torque);
    // Vofa_JustFloat.Bind_Channel(6,&w_1_filter);
    // Vofa_JustFloat.Bind_Channel(7,&w_2_filter);
    // Vofa_JustFloat.Bind_Channel(8,&w_3_filter);
    Vofa_JustFloat.Bind_Channel(0,&a);
    Vofa_JustFloat.Bind_Channel(1,&b);
    Vofa_JustFloat.Bind_Channel(2,&c);
    Vofa_JustFloat.Bind_Channel(3,&d);
    Vofa_JustFloat.Bind_Channel(4,&e);
    Vofa_JustFloat.Bind_Channel(5,&f);
    Vofa_JustFloat.Bind_Channel(6,&g);
    Vofa_JustFloat.Bind_Channel(7,&q[3]);




}

/**
 *@brief VOFA+ 串口绘图slider初始化
 *
 */
void Vofa_Tuner_Init()
{
    Vofa_Tuner.Init(&huart7,64);

    Vofa_Tuner.Set_Reset_Integral_On_Tune(1);

    //绑定
    Vofa_Tuner.Bind_PID(1000,&robot.Shooter.Motor_Driver.PID_Angle);
    // Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Track_Wheel[0].PID_Omega);
    // Vofa_Tuner.Bind_PID(1100,&robot.Chassis.Track_Wheel[1].PID_Omega);
    //Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Track_Wheel[0].PID_Omega);
}

/**
 *@brief UART1 VOFA+回调函数
 *
 */
void Vofa_JustFloat_UART1_Callback()
{
    a = robot.AutoAim.Get_Rx_Mode();
    b = robot.AutoAim.Get_Target_Pitch();
    c = robot.AutoAim.Get_Target_Yaw();
    // a = robot.Shooter.Motor_Driver.Get_Target_Angle();
    // b = robot.Shooter.Motor_Driver.Get_Now_Angle();
    // c = robot.Shooter.Motor_Driver.Get_Target_Omega();
    // d = robot.Shooter.Motor_Driver.Get_Now_Filter_Omega();
    // e = robot.Shooter.Motor_Driver.Get_Target_Torque();
    // f = robot.Shooter.Motor_Driver.Get_Now_Torque();
    // a = robot.Slave_With_Master.Get_Chassis_Power_Max();
    // b = robot.referee.Get_Self_Booster_Heat_Max();
    // c = robot.referee.Get_Booster_42mm_Heat();
    // d = robot.referee.Get_Status();
    // a = robot.Gimbal.Board_IMU.Get_GyroOffset_X();
    // b = robot.Gimbal.Board_IMU.Get_GyroOffset_Y();
    // c = robot.Gimbal.Board_IMU.Get_GyroOffset_Z();
    // d = robot.Gimbal.Board_IMU.Get_IsStationary();
    // e = robot.Gimbal.Board_IMU.Get_GyroNorm();
    // f = robot.Gimbal.Board_IMU.Get_AccelNorm();
    // g = robot.Gimbal.Board_IMU.Get_AccelError();
    // a = robot.Gimbal.Board_IMU.Get_Yaw();
    // b = robot.Gimbal.Board_IMU.Get_Pitch();
    // c = robot.Gimbal.Board_IMU.Get_Roll();
    // d = robot.Gimbal.Board_IMU.Get_Q4();
    // a = robot.Gimbal.Yaw_Motor_MIT.Get_Now_Angle();
    // b = robot.Gimbal.Get_Now_Yaw_Relative_Angle();
    // c = robot.Gimbal.Get_Now_Yaw_Angle();
    // d = robot.Gimbal.Yaw_Motor_MIT.Get_Now_Angle();
    // d = robot.Slave_With_Master.Get_Chassis_Power_Max();
    //
    // b = robot.Gimbal.Pitch_Motor_MIT.Get_Now_Angle();
    // c = robot.Gimbal.Pitch_Motor_MIT.Get_Status();
    // d = robot.AutoAim.Get_Rx_Mode();
    // P = robot.Gimbal.Get_Now_Pitch_Angle();
    // Y = robot.Gimbal.Get_Now_Yaw_Angle();
    // tar_angle = robot.Gimbal.Get_Target_Yaw_Angle();
    // tar_omega = robot.Gimbal.Class_PID_Yaw_Angle_IMU.Get_Out();
    // tar_torque = robot.Gimbal.Class_PID_Yaw_Omega_IMU.Get_Out();
    // now_angle = robot.Gimbal.Get_Now_Pitch_Angle();
    // now_omega = robot.Gimbal.Get_Now_Yaw_Omega();
    // // now_torque = robot.Gimbal.Yaw_Motor_MIT.Get_Now_Torque() * 1.0f;
    // now_torque = robot.Gimbal.Get_Omega();


    // tar_angle = robot.AutoAim.Get_Target_Pitch();
    // now_angle = robot.Gimbal.Get_Now_Pitch_Angle();
    // tar_omega = robot.AutoAim.Get_Target_Yaw();
    // now_omega = robot.Gimbal.Get_Now_Yaw_Angle();
    // tar_torque = robot.Gimbal.Get_Target_Pitch_Angle();
    // now_torque = robot.Gimbal.Get_Target_Yaw_Angle();

    // tar_angle = robot.Shooter.Motor_Driver.Get_Target_Angle();
    // now_angle = robot.Shooter.Motor_Driver.Get_Now_Angle_Continuous();
    // tar_omega = robot.Shooter.Motor_Driver.PID_Angle.Get_Out();
    // now_omega = robot.Shooter.Motor_Driver.Get_Now_Filter_Omega();
    // tar_torque = robot.Shooter.Motor_Driver.Get_Target_Torque();
    // now_torque = robot.Shooter.Motor_Driver.Get_Now_Torque();

    // tar_w_1 =21.0f;
    // w_1_filter = robot.Shooter.Motor_Friction_1.Get_Now_Filter_Omega();
    // w_2_filter = robot.Shooter.Motor_Friction_2.Get_Now_Filter_Omega();
    // w_3_filter = robot.Shooter.Motor_Friction_3.Get_Now_Filter_Omega();
    // tar_torque = robot.Shooter.Motor_Friction_1.PID_Omega.Get_Out();
    // now_torque = robot.Shooter.Motor_Friction_1.Get_Now_Torque();
    //
    // w_1 = robot.Shooter.Motor_Friction_1.Get_Now_Omega();
    //
    // w_1_current = robot.Shooter.Motor_Friction_1.Get_Target_Torque();
    // w_1_now_current = robot.Shooter.Motor_Friction_1.Get_Now_Torque();


    // pitch = robot.Gimbal.DM_IMU.Get_Pitch_Deg();
    // yaw = robot.Gimbal.DM_IMU.Get_Yaw_Deg();
    // roll = robot.Gimbal.DM_IMU.Get_Roll_Deg();
    // P = robot.Gimbal.Get_Now_Pitch_Angle();
    // Y = robot.Gimbal.Get_Now_Yaw_Angle();
    // vt = robot.VT13.Get_Right_X();
    // target_angle = robot.Gimbal.Get_Target_Pitch_Angle();
    // target_omega = robot.Gimbal.Class_PID_Pitch_Angle_IMU.Get_Out();
    // target_torque = robot.Gimbal.Class_PID_Pitch_Omega_IMU.Get_Out();
    // now_angle = robot.Gimbal.Get_Now_Pitch_Angle();
    // // now_angle = robot.Gimbal.Pitch_Motor_MIT.Get_Now_Angle();
    //
    // now_omega = robot.Gimbal.Get_Now_Pitch_Omega();
    // now_torque = robot.Gimbal.Pitch_Motor_MIT.Get_Now_Torque() * 1.0f;
    //
    // // pitch = robot.Gimbal.DM_IMU.Get_Pitch_Deg();
    // // yaw = robot.Gimbal.DM_IMU.Get_Yaw_Deg();
    // // roll = robot.Gimbal.DM_IMU.Get_Roll_Deg();
    // // P = robot.Gimbal.Get_Now_Pitch_Angle();
    // // Y = robot.Gimbal.Get_Now_Yaw_Angle();
    // // vt = robot.VT13.Get_Right_Y();
    // // target_angle = robot.Gimbal.Get_Target_Yaw_Angle();
    // // target_omega = robot.Gimbal.Class_PID_Yaw_Angle_IMU.Get_Out();
    // // // target_omega = 3.0;
    // //
    // // target_torque = robot.Gimbal.Class_PID_Yaw_Omega_IMU.Get_Out();
    // // now_angle = robot.Gimbal.Get_Now_Yaw_Angle();
    // // now_omega = robot.Gimbal.Get_Now_Yaw_Omega();
    // // now_torque = robot.Gimbal.Yaw_Motor_MIT.Get_Now_Torque() * 1.0f;


    // PID
    // kp = robot.Chassis.Track_Wheel[0].PID_Omega.Get_K_P();
    // ki = robot.Chassis.Track_Wheel[0].PID_Omega.Get_K_I();
    // kd = robot.Chassis.Track_Wheel[0].PID_Omega.Get_K_D();
    // kf = robot.Chassis.Track_Wheel[0].PID_Omega.Get_K_F();
    // i_out_max  = robot.Chassis.Track_Wheel[0].PID_Omega.Get_I_Out_Max();
    // out_max = robot.Chassis.Track_Wheel[0].PID_Omega.Get_Out_Max();

    // kp = robot.Chassis.PID_Vx.Get_K_P();
    // ki = robot.Chassis.PID_Vx.Get_K_I();
    // kd = robot.Chassis.PID_Vx.Get_K_D();
    // kf = robot.Chassis.PID_Vx.Get_K_F();
    // i_out_max  = robot.Chassis.PID_Vx.Get_I_Out_Max();
    // out_max = robot.Chassis.PID_Vx.Get_Out_Max();

    // 底盘

    // 麦轮底盘目标转速


    // 麦轮底盘转速
    // Chassis_Motor_0_Omega = robot.Chassis.Motor_Wheel[0].Get_Now_Omega();
    // Chassis_Motor_1_Omega = robot.Chassis.Motor_Wheel[1].Get_Now_Omega()*(-1.0f);
    // Chassis_Motor_2_Omega = robot.Chassis.Motor_Wheel[2].Get_Now_Omega()*(-1.0f);
    // Chassis_Motor_3_Omega = robot.Chassis.Motor_Wheel[3].Get_Now_Omega();
    // 履带电机转速
    // Track_Motor_0_Omega = robot.Chassis.Track_Wheel[0].Get_Now_Omega();
    // Track_Motor_1_Omega = robot.Chassis.Track_Wheel[1].Get_Now_Omega();
    // 麦轮目标转速
    // Chassis_Motor_0_Tar_Omega = robot.Chassis.Motor_Wheel[0].Get_Target_Omega();
    // Chassis_Motor_1_Tar_Omega = robot.Chassis.Motor_Wheel[1].Get_Target_Omega()*(-1.0f);
    // Chassis_Motor_2_Tar_Omega = robot.Chassis.Motor_Wheel[2].Get_Target_Omega()*(-1.0f);
    // Chassis_Motor_3_Tar_Omega = robot.Chassis.Motor_Wheel[3].Get_Target_Omega();
    // 履带电机目标转速
    // Track_Motor_0_Tar_Omega = robot.Chassis.Track_Wheel[0].Get_Target_Omega();
    // 麦轮底盘电流
    // Chassis_Motor_0_Current = robot.Chassis.Motor_Wheel[0].Get_Now_Current();
    // Chassis_Motor_1_Current = robot.Chassis.Motor_Wheel[1].Get_Now_Current();
    // Chassis_Motor_2_Current = robot.Chassis.Motor_Wheel[2].Get_Now_Current();
    // Chassis_Motor_3_Current = robot.Chassis.Motor_Wheel[3].Get_Now_Current();
    // 履带电机电流
    // Track_Motor_0_Current = robot.Chassis.Track_Wheel[0].Get_Now_Current();
    // Track_Motor_1_Current = robot.Chassis.Track_Wheel[1].Get_Now_Current();

    // 车体目标速度
    // Vx = robot.VT13.Get_Left_X()*3.0f;
    // Vy = -robot.VT13.Get_Left_Y()*3.0f;
    // Omega = -robot.VT13.Get_Right_Y()*1.0f*PI;
    // 车体真实速度
    // measure_vx = (Chassis_Motor_0_Omega + Chassis_Motor_1_Omega + Chassis_Motor_2_Omega + Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f;
    // measure_vy = (Chassis_Motor_0_Omega - Chassis_Motor_1_Omega - Chassis_Motor_2_Omega + Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f;
    // measure_omega = (Chassis_Motor_0_Omega - Chassis_Motor_1_Omega + Chassis_Motor_2_Omega - Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f/(0.92f/2.0f);

    // 麦轮目标电流
    // Chassis_Motor_0_Tar_Omega = robot.Chassis.Motor_Wheel[0].Get_Target_Omega();
    // Chassis_Motor_1_Tar_Omega = robot.Chassis.Motor_Wheel[1].Get_Target_Omega();
    // Chassis_Motor_2_Tar_Omega = robot.Chassis.Motor_Wheel[2].Get_Target_Omega();
    // Chassis_Motor_3_Tar_Omega = robot.Chassis.Motor_Wheel[3].Get_Target_Omega();

    // 底盘功率调试
    // chassis_power_cmd_est     = robot.Chassis.Get_Estimated_Power_Cmd();
    // chassis_power_limited_est = robot.Chassis.Get_Estimated_Power_Limited();
    // chassis_power_limit_max_dbg = 8.0f;
}

/**
 *@brief UART1 VOFA+调参回调函数
 *
 *@param Buffer UART1收到的消息
 *@param Length 长度
 */
void Vofa_Tuner_UART1_Callback(uint8_t *Buffer, uint16_t Length)
{
    Vofa_Tuner.UART_RxCpltCallback(Buffer, Length);
}


/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}


/**
 *@brief CAN1回调函数
 *
 *@param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;
    switch (Header.Identifier)
    {
    case 0x201: robot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x202: robot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(); break;
    case 0x203: robot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(); break;
    case 0x204: robot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(); break;
    case 0x205: robot.Chassis.Track_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x206: robot.Chassis.Track_Wheel[1].CAN_RxCpltCallback(); break;

    case 0x310:
    case 0x311:
    case 0x312:
        robot.Slave_With_Master.CAN_RxCpltCallback(Header, Buffer);
        break;
    default: break;
    }
}

/**
 *@brief CAN2回调函数
 *
 *@param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;
    switch (Header.Identifier)
    {
    case 0x00: robot.Chassis.Leg_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x01: robot.Chassis.Leg_Wheel[1].CAN_RxCpltCallback(); break;
    case 0x02: robot.Gimbal.Yaw_Motor_MIT.CAN_RxCpltCallback(); break;
    case 0x03: robot.Shooter.Motor_Driver.CAN_RxCpltCallback(); break;

    default: break;
    }
}

/**
 *@brief CAN3回调函数
 *
 *@param CAN_RxMessage CAN3收到的消息
 */
void Device_CAN3_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;
    switch (Header.Identifier)
    {
    case 0x00: robot.Gimbal.Pitch_Motor_MIT.CAN_RxCpltCallback(); break;
    case 0x201: robot.Shooter.Motor_Friction_1.CAN_RxCpltCallback(); break;
    case 0x202: robot.Shooter.Motor_Friction_2.CAN_RxCpltCallback(); break;
    case 0x203: robot.Shooter.Motor_Friction_3.CAN_RxCpltCallback(); break;

    default: break;
    }
}


/**
 *@brief UART1遥控器回调函数
 *
 *@param Buffer UART1收到的消息
 *@param Length 长度
 *
 */
void Image_Transform_UART1_Callback(uint8_t *Buffer,uint16_t Length)
{
    robot.VT13.UART_RxCpltCallback(Buffer,Length);
}

/**
 *@brief 虚拟串口上位机接收回调
 *
 */
void AutoAim_USB_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (!init_finished)
    {
        return;
    }

    // 达妙云台还没反馈正常前，不处理视觉数据
    // 避免上电阶段 USB 高频数据影响 CAN / DM 使能过程
    if (robot.Gimbal.Yaw_Motor_MIT.Get_Status() != Motor_DM_Status_ENABLE ||
        robot.Gimbal.Pitch_Motor_MIT.Get_Status() != Motor_DM_Status_ENABLE)
    {
        return;
    }

    robot.AutoAim.USB_RxCpltCallback(Buffer, Length);
}

/**
 *@brief 裁判系统回调函数UART6
 *
 */
// void Referee_UART6_Callback(uint8_t *Buffer,uint16_t Length)
// {
//     robot.referee.UART_RxCpltCallback(Buffer,Length);
// }

/**
 *@brief TIM4任务回调函数
 *
 */
void Task100us_TIM4_Callback()
{

}

/**
 *@brief TIM5任务回调函数
 *
 */
void Task1ms_TIM7_Callback()
{
    // 模块保持存活

    static int alive_mod100 = 0;
    alive_mod100++;
    if (alive_mod100 == 100)
    {
        alive_mod100 = 0;
        robot.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    static int alive_mod1000 = 0;
    alive_mod1000++;
    if (alive_mod1000 == 1000)
    {
        alive_mod1000 = 0;
        robot.TIM_1000ms_Alive_PeriodElapsedCallback();
    }

    static int interaction_mod200 = 0;
    interaction_mod200++;
    if (interaction_mod200 == 200)
    {
        interaction_mod200 = 0;
        robot.TIM_200ms_Alive_PeriodElapsedCallback();
    }

    static int interaction_mod100 = 0;
    interaction_mod100++;
    if (interaction_mod100 == 100)
    {
        interaction_mod100 = 0;
        robot.TIM_100ms_Calculate_Callback();
    }

    static int interaction_mod10 = 0;
    interaction_mod10++;
    if (interaction_mod10 == 10)
    {
        interaction_mod10 = 0;
        robot.TIM_10ms_Calculate_PeriodElapsedCallback();
    }

    static int interaction_mod2 = 0;
    interaction_mod2++;
    if (interaction_mod2 == 2)
    {
        interaction_mod2 = 0;
        robot.TIM_2ms_Calculate_PeriodElapsedCallback();
    }

    robot.TIM_1ms_Calculate_Callback();

    // 设备层回调函数
    Vofa_JustFloat_UART1_Callback();
    Vofa_JustFloat.TIM_1ms_PeriodElapsedCallback();

    // 驱动层回调函数
    TIM_1ms_CAN_PeriodElapsedCallback();

    flag++;
}

/**
 *@brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);
    // 驱动层初始化

    // CAN总线初始化
    CAN_Init(&hfdcan1,Device_CAN1_Callback);
    CAN_Init(&hfdcan2,Device_CAN2_Callback);
    CAN_Init(&hfdcan3,Device_CAN3_Callback);

    // // UART初始化
    // UART_Init(&huart3, Referee_UART6_Callback, 128);                //裁判系统
    UART_Init(&huart1,Image_Transform_UART1_Callback,84);           //遥控器
    UART_Init(&huart7,Vofa_Tuner_UART1_Callback,64);                //VOFA
    USB_Init(AutoAim_USB_Callback);

    // 定时器中断初始化
    HAL_TIM_Base_Start(&htim5);

    // 设备层初始化

    // 牢雄初始化
    robot.Init();

    // VOFA+串口绘图初始化
    Vofa_JustFloat_Init();
    Vofa_Tuner_Init();



    // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim7);                  //1ms调度

    // 等待系统
    HAL_Delay(2000);
    // 标记初始化完成
    init_finished = true;

}

/**
 *@brief 前台循环任务
 *
 */
void Task_Loop()
{

}



/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_TIM7_Callback();
    }

}