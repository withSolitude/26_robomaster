//
// Created by Lenovo on 2026/1/12.
//
#include "5_Task/tsk_config_and_callback.h"
#include "4_Interaction/ita_robot.h"
#include "2_Device/Vofa/dvc_vofa.h"
#include "1_Middleware/1_Driver/TIM/drv_tim.h"
#include "3_Robot/Chassis/Chassis_Control.h"
#include "Vofa/dvc_Vofa_Tuner.h"
#include "1_Driver/CAN/drv_can.h"
#include "IMU/DM_IMU/dvc_dm_imu.h"

// 调试

float yaw;
float pitch;
float roll;

// 功率上限
float Chassis_Power_Max;
// 缓冲能量
float Chassis_Energy_Buffer;
// 17mm热量
float Booster_17mm_Heat;
// 42mm热量
float Booster_42mm_Heat;

bool init_finished = false;
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
    Vofa_JustFloat.Init(&huart1,3,20);
    //绑定通道
    //Vofa_JustFloat.Bind_Channel(1,&chassis_pitch_angle);
    //Vofa_JustFloat.Bind_Channel(2,&now_right_leg_angle);
    // Vofa_JustFloat.Bind_Channel(0,&Chassis_Power_Max);
    // Vofa_JustFloat.Bind_Channel(1,&Chassis_Energy_Buffer);
    // Vofa_JustFloat.Bind_Channel(2,&Booster_17mm_Heat);
    // Vofa_JustFloat.Bind_Channel(3,&Booster_42mm_Heat);
    Vofa_JustFloat.Bind_Channel(0,&Chassis_Power_Max);
    Vofa_JustFloat.Bind_Channel(1,&Chassis_Energy_Buffer);
    Vofa_JustFloat.Bind_Channel(2,&Booster_42mm_Heat);


}

/**
 *@brief VOFA+ 串口绘图slider初始化
 *
 */
void Vofa_Tuner_Init()
{
    Vofa_Tuner.Init(&huart1,64);

    Vofa_Tuner.Set_Reset_Integral_On_Tune(1);

}

/**
 *@brief UART1 VOFA+回调函数
 *
 */
void Vofa_JustFloat_UART1_Callback()
{
    Chassis_Power_Max = robot.referee.Get_Self_Chassis_Power_Max();
    Chassis_Energy_Buffer = robot.referee.Get_Chassis_Energy_Buffer();
    Booster_17mm_Heat = robot.referee.Get_Booster_17mm_Heat();
    Booster_42mm_Heat = robot.referee.Get_Booster_42mm_Heat();
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
 *@brief CAN1回调函数
 *
 *@param CAN_RxMessage CAN1收到的消息
 */
void Device_CAN1_Callback(Struct_CAN_Rx_Buffer * CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {

    }
}

/**
 *@brief CAN2回调函数
 *
 *@param CAN_RxMessage CAN2收到的消息
 */
void Device_CAN2_Callback(Struct_CAN_Rx_Buffer * CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    }
}

/**
 *@brief 裁判系统回调函数UART6
 *
 */
void Referee_UART6_Callback(uint8_t *Buffer,uint16_t Length)
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
void Task1ms_TIM5_Callback()
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
    Vofa_JustFloat_UART1_Callback();
    Vofa_JustFloat.TIM_1ms_PeriodElapsedCallback();

    // 驱动层回调函数
    TIM_1ms_CAN_PeriodElapsedCallback();
    TIM_1ms_UART_PeriodElapsedCallback();

    flag++;
}

/**
 *@brief 初始化任务
 *
 */
void Task_Init()
{
    // 驱动层初始化

    // CAN总线初始化
    CAN_Init(&hcan1,Device_CAN1_Callback);
    CAN_Init(&hcan2,Device_CAN2_Callback);
    // UART初始化
    UART_Init(&huart6, Referee_UART6_Callback, 512);                //裁判系统
    UART_Init(&huart1,Vofa_Tuner_UART1_Callback,64);                //VOFA

    // 定时器初始化
    TIM_Init(&htim4,Task100us_TIM4_Callback);
    TIM_Init(&htim5,Task1ms_TIM5_Callback);

    // 设备层初始化

    // 牢雄初始化
    robot.Init();

    // VOFA+串口绘图初始化
    Vofa_JustFloat_Init();
    Vofa_Tuner_Init();

    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);

    // 标记初始化完成
    init_finished = true;

    // 等待系统
    HAL_Delay(2000);
}

/**
 *@brief 前台循环任务
 *
 */
void Task_Loop()
{

}

