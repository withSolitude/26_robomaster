//
// Created by Lenovo on 2026/1/12.
//
#include "5_Task/tsk_config_and_callback.h"
#include "4_Interaction/ita_robot.h"
#include "2_Device/Vofa/dvc_vofa.h"
//#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "3_Robot/Chassis/Chassis_Control.h"
#include "2_Device/Vofa/dvc_Vofa_Tuner.h"
#include "2_Device/DM_IMU/dvc_dm_imu.h"

float a;
float b;
float c;



//调试专用

float out;

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
// 腿电机目标转速
float Leg_Motor_0_Tar_Omega;
float Leg_Motor_1_Tar_Omega;

// 麦轮底盘电机转速
float Chassis_Motor_0_Omega;
float Chassis_Motor_1_Omega;
float Chassis_Motor_2_Omega;
float Chassis_Motor_3_Omega;
// 履带3508电机转速
float Track_Motor_0_Omega;
float Track_Motor_1_Omega;
// 腿电机转速
float Leg_Motor_0_Omega;
float Leg_Motor_1_Omega;

// 麦轮目标底盘电机电流
float Chassis_Motor_Tar_0_Current;
float Chassis_Motor_Tar_1_Current;
float Chassis_Motor_Tar_2_Current;
float Chassis_Motor_Tar_3_Current;
// 腿电机目标电流
float Leg_Motor_Tar_0_Current;
float Leg_Motor_Tar_1_Current;

// 麦轮底盘电机电流
float Chassis_Motor_0_Current;
float Chassis_Motor_1_Current;
float Chassis_Motor_2_Current;
float Chassis_Motor_3_Current;
// 履带3508电机电流
float Track_Motor_0_Current;
float Track_Motor_1_Current;
// 腿电机电流
float Leg_Motor_0_Current;
float Leg_Motor_1_Current;

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
Class_DM_IMU DM_IMU;
/**
 *@brief VOFA+ 串口绘图UART1初始化
 *
 */
void Vofa_JustFloat_Init(void)
{
    Vofa_JustFloat.Init(&huart7,7,20);
    //绑定通道
    Vofa_JustFloat.Bind_Channel(0,&chassis_power_cmd_est);
    Vofa_JustFloat.Bind_Channel(1,&chassis_power_limited_est);
    Vofa_JustFloat.Bind_Channel(2,&Leg_Motor_0_Tar_Omega);
    Vofa_JustFloat.Bind_Channel(3,&Leg_Motor_0_Omega);
    Vofa_JustFloat.Bind_Channel(4,&Leg_Motor_Tar_0_Current);
    Vofa_JustFloat.Bind_Channel(5,&Leg_Motor_0_Current);
    Vofa_JustFloat.Bind_Channel(6,&c);
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
    Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Track_Wheel[0].PID_Omega);

}

/**
 *@brief UART1 VOFA+回调函数
 *
 */
void Vofa_JustFloat_UART7_Callback()
{
    a = robot.Chassis.Get_Target_Chassis_Leg_Motor_Angle();
    b = robot.Chassis.Leg_Wheel[0].Get_Now_Angle();

    Leg_Motor_0_Tar_Omega = robot.Chassis.Leg_Wheel[0].PID_Angle.Get_Out();
    Leg_Motor_0_Tar_Omega = robot.Chassis.Leg_Wheel[1].PID_Angle.Get_Out();

    Leg_Motor_0_Omega = robot.Chassis.Leg_Wheel[0].Get_Now_Omega();
    Leg_Motor_1_Omega = robot.Chassis.Leg_Wheel[1].Get_Now_Omega();

    Leg_Motor_Tar_0_Current = robot.Chassis.Get_Target_Leg_Motor_Torque();
    Leg_Motor_Tar_1_Current = robot.Chassis.Leg_Wheel[1].PID_Omega.Get_Out();

    Leg_Motor_0_Current = robot.Chassis.Leg_Wheel[0].Get_Now_Torque();
    Leg_Motor_1_Current = robot.Chassis.Leg_Wheel[1].Get_Now_Torque();

    c = -robot.Chassis.Leg_Wheel[1].Get_Now_Angle();

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
    // Chassis_Motor_0_Current = robot.Chassis.Motor_Wheel[0].Get_Now_Torque();
    // Chassis_Motor_1_Current = robot.Chassis.Motor_Wheel[1].Get_Now_Torque();
    // Chassis_Motor_2_Current = robot.Chassis.Motor_Wheel[2].Get_Now_Torque();
    // Chassis_Motor_3_Current = robot.Chassis.Motor_Wheel[3].Get_Now_Torque();
    // // 履带电机电流
    // // Track_Motor_0_Current = robot.Chassis.Track_Wheel[0].Get_Now_Current();
    // // Track_Motor_1_Current = robot.Chassis.Track_Wheel[1].Get_Now_Current();
    //
    // // 车体目标速度
    // Vx = robot.VT13.Get_Left_X()*2.0f;
    // Vy = -robot.VT13.Get_Left_Y()*2.0f;
    // Omega = -robot.VT13.Get_Right_Y()*2.0f*PI;
    // //车体真实速度
    // measure_vx = (Chassis_Motor_0_Omega + Chassis_Motor_1_Omega + Chassis_Motor_2_Omega + Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f;
    // measure_vy = (Chassis_Motor_0_Omega - Chassis_Motor_1_Omega + Chassis_Motor_2_Omega + Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f;
    // measure_omega = (-Chassis_Motor_0_Omega + Chassis_Motor_1_Omega + Chassis_Motor_2_Omega - Chassis_Motor_3_Omega)*0.1524977f* 0.25f * 0.70710678118f/(0.92f/2.0f);
    //
    // // 麦轮目标电流
    // Chassis_Motor_Tar_0_Current = robot.Chassis.Motor_Wheel[0].Get_Target_Torque();
    // Chassis_Motor_Tar_1_Current = robot.Chassis.Motor_Wheel[1].Get_Target_Torque();
    // Chassis_Motor_Tar_2_Current = robot.Chassis.Motor_Wheel[2].Get_Target_Torque();
    // Chassis_Motor_Tar_3_Current = robot.Chassis.Motor_Wheel[3].Get_Target_Torque();

    // 底盘功率调试
    chassis_power_cmd_est     = robot.Chassis.Get_Estimated_Power_Cmd();
    chassis_power_limited_est = robot.Chassis.Get_Estimated_Power_Limited();
    chassis_power_limit_max_dbg = 8.0f;

}

/**
 *@brief UART1 VOFA+调参回调函数
 *
 *@param Buffer UART1收到的消息
 *@param Length 长度
 */
void Vofa_Tuner_UART7_Callback(uint8_t *Buffer, uint16_t Length)
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
    case 0x310:
        robot.Slave_With_Master.CAN_RxCpltCallback(Header, Buffer);
        break;
    default: break;
    }
}

/**
 *@brief CAN1回调函数
 *
 *@param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;
    switch (Header.Identifier)
    {
    case 0x205: robot.Chassis.Motor_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x206: robot.Chassis.Motor_Wheel[1].CAN_RxCpltCallback(); break;
    case 0x207: robot.Chassis.Motor_Wheel[2].CAN_RxCpltCallback(); break;
    case 0x208: robot.Chassis.Motor_Wheel[3].CAN_RxCpltCallback(); break;
    case 0x201: robot.Chassis.Track_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x202: robot.Chassis.Track_Wheel[1].CAN_RxCpltCallback(); break;
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
    case 0x00: robot.Chassis.Leg_Wheel[0].CAN_RxCpltCallback(); break;
    case 0x01: robot.Chassis.Leg_Wheel[1].CAN_RxCpltCallback(); break;
   // case 0x05: DM_IMU.CAN_RxCpltCallback(Buffer); break;
    default: break;
    }
}

/**
 *@brief UART6遥控器回调函数
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
 *@brief 裁判系统回调函数UART1
 *
 */
void Referee_UART1_Callback(uint8_t *Buffer,uint16_t Length)
{
    robot.referee.UART_RxCpltCallback(Buffer,Length);
}

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
    Vofa_JustFloat_UART7_Callback();
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
    // UART_Init(&huart1,Image_Transform_UART1_Callback,84);           //遥控器

    // 定时器初始化

//     // 定时器中断初始化
    HAL_TIM_Base_Start(&htim5);

    // 设备层初始化

    // 牢雄初始化
    robot.Init();

    // VOFA+串口绘图初始化
    Vofa_JustFloat_Init();
    Vofa_Tuner_Init();

    UART_Init(&huart1,Referee_UART1_Callback,512);           //遥控器
   UART_Init(&huart7,Vofa_Tuner_UART7_Callback,64);                //VOFA

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