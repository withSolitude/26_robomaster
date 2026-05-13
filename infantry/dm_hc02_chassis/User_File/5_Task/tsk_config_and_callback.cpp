// /**
//  * @file tsk_config_and_callback.cpp
//  * @author yssickjgd (1345578933@qq.com)
//  * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
//  * @version 0.1
//  * @date 2023-08-29 0.1 23赛季定稿
//  * @date 2023-01-17 1.1 调试到机器人层
//  *
//  * @copyright USTC-RoboWalker (c) 2023-2024
//  *
//  */
//
// /* Includes ------------------------------------------------------------------*/
//
// #include "tsk_config_and_callback.h"
//
// #include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
// #include "2_Device/BSP/BMI088/bsp_bmi088.h"
// #include "2_Device/Vofa/dvc_vofa.h"
// #include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
// #include "2_Device/BSP/WS2812/bsp_ws2812.h"
// #include "2_Device/BSP/Buzzer/bsp_buzzer.h"
// #include "2_Device/BSP/Power/bsp_power.h"
// #include "2_Device/BSP/Key/bsp_key.h"
// #include "1_Middleware/2_Algorithm/Filter/Kalman/alg_filter_kalman.h"
// #include "1_Middleware/2_Algorithm/Matrix/alg_matrix.h"
// #include "1_Middleware/1_Driver/WDG/drv_wdg.h"
// #include "1_Middleware/3_System/Timestamp/sys_timestamp.h"
//
// /* Private macros ------------------------------------------------------------*/
//
// /* Private types -------------------------------------------------------------*/
//
// /* Private variables ---------------------------------------------------------*/
//
// // 串口绘图
// char Vofa_Variable_Assignment_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {"q00", "q11", "r00", "r11",};
//
// // LED灯
// uint8_t red = 0;
// uint8_t green = 170;
// uint8_t blue = 170;
// bool red_minus_flag = false;
// bool green_minus_flag = false;
// bool blue_minus_flag = true;
//
// // 大疆电机3508
// Class_Motor_DJI_GM6020 motor;
// // Kalman滤波器
// Class_Filter_Kalman filter_kalman;
// // 相关矩阵
// Class_Matrix_f32<2, 2> A;
// Class_Matrix_f32<2, 1> B;
// Class_Matrix_f32<2, 2> H;
// Class_Matrix_f32<2, 2> Q;
// Class_Matrix_f32<2, 2> R;
// Class_Matrix_f32<2, 2> P;
//
// // 全局初始化完成标志位
// bool init_finished = false;
//
// /* Private function declarations ---------------------------------------------*/
//
// /* Function prototypes -------------------------------------------------------*/
//
// /**
//  * @brief USB虚拟串口接收回调函数
//  *
//  * @param Buffer 接收缓冲区
//  * @param Length 接收数据长度
//  */
// void Serial_USB_Call_Back(uint8_t *Buffer, uint16_t Length)
// {
//     Vofa_USB.USB_RxCallback(Buffer, Length);
//     int32_t index = Vofa_USB.Get_Variable_Index();
//     switch (index)
//     {
//     case (0):
//     {
//         filter_kalman.Matrix_Q[0][0] = Vofa_USB.Get_Variable_Value();
//         break;
//     }
//     case (1):
//     {
//         filter_kalman.Matrix_Q[1][1] = Vofa_USB.Get_Variable_Value();
//         break;
//     }
//     case (2):
//     {
//         filter_kalman.Matrix_R[0][0] = Vofa_USB.Get_Variable_Value();
//         break;
//     }
//     case (3):
//     {
//         filter_kalman.Matrix_R[1][1] = Vofa_USB.Get_Variable_Value();
//         break;
//     }
//     default:
//     {
//         break;
//     }
//     }
// }
//
// /**
//  * @brief SPI2任务回调函数
//  *
//  */
// void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
// {
//     if (SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin || SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port && SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin)
//     {
//         BSP_BMI088.SPI_RxCpltCallback();
//     }
// }
//
// /**
//  * @brief CAN1回调函数
//  *
//  *
//  */
// void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
// {
//     switch (Header.Identifier)
//     {
//     case (0x206):
//     {
//         motor.CAN_RxCpltCallback();
//
//         break;
//     }
//     }
// }
//
// /**
//  * @brief OSPI2轮询回调函数
//  *
//  */
// void OSPI2_Polling_Callback()
// {
//     BSP_W25Q64JV.OSPI_StatusMatchCallback();
// }
//
// /**
//  * @brief OSPI2接收回调函数
//  *
//  */
// void OSPI2_Rx_Callback(uint8_t *Buffer, uint16_t Length)
// {
//     BSP_W25Q64JV.OSPI_RxCallback();
// }
//
// /**
//  * @brief 每3600s调用一次
//  *
//  */
// void Task3600s_Callback()
// {
//     SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
// }
//
// /**
//  * @brief 每1s调用一次
//  *
//  */
// void Task1s_Callback()
// {
// }
//
// /**
//  * @brief 每1ms调用一次
//  *
//  */
// void Task1ms_Callback()
// {
//     static int mod10 = 0;
//     mod10++;
//     if (mod10 == 10)
//     {
//         mod10 = 0;
//
//         if (red == 10)
//         {
//             red_minus_flag = true;
//         }
//         else if (red == 0)
//         {
//             red_minus_flag = false;
//         }
//         if (green == 10)
//         {
//             green_minus_flag = true;
//         }
//         else if (green == 0)
//         {
//             green_minus_flag = false;
//         }
//         if (blue == 10)
//         {
//             blue_minus_flag = true;
//         }
//         else if (blue == 0)
//         {
//             blue_minus_flag = false;
//         }
//
//         if (red_minus_flag)
//         {
//             red--;
//         }
//         else
//         {
//             red++;
//         }
//         if (green_minus_flag)
//         {
//             green--;
//         }
//         else
//         {
//             green++;
//         }
//         if (blue_minus_flag)
//         {
//             blue--;
//         }
//         else
//         {
//             blue++;
//         }
//
//         BSP_WS2812.Set_RGB(red, green, blue);
//         // BSP_WS2812.Set_RGB(0, 0, 0);
//
//         // 发送实例
//         BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
//     }
//
//     BSP_Buzzer.Set_Sound(0.0f, 0.0f);
//
//     BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
//     static int mod50 = 0;
//     mod50++;
//     if (mod50 == 50)
//     {
//         mod50 = 0;
//
//         // 处理按键状态
//         BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
//     }
//
//     static int mod100 = 0;
//     mod100++;
//     if (mod100 == 100)
//     {
//         mod100 = 0;
//
//         motor.TIM_100ms_Alive_PeriodElapsedCallback();
//     }
//     motor.Set_Target_Angle(1.0f * PI);
//     motor.TIM_Calculate_PeriodElapsedCallback();
//
//     static int mod128 = 0;
//     mod128++;
//     if (mod128 == 128)
//     {
//         mod128 = 0;
//
//         BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
//     }
//
//     filter_kalman.Vector_Z[0][0] = motor.Get_Now_Angle();
//     filter_kalman.Vector_Z[1][0] = motor.Get_Now_Omega();
//     filter_kalman.TIM_Predict_PeriodElapsedCallback();
//     filter_kalman.TIM_Update_PeriodElapsedCallback();
//
//     float yaw = BSP_BMI088.Get_Euler_Angle()[0][0] / BASIC_MATH_DEG_TO_RAD;
//     float pitch = BSP_BMI088.Get_Euler_Angle()[1][0] / BASIC_MATH_DEG_TO_RAD;
//     float roll = BSP_BMI088.Get_Euler_Angle()[2][0] / BASIC_MATH_DEG_TO_RAD;
//     float q0 = BSP_BMI088.Get_Quaternion()[0];
//     float q1 = BSP_BMI088.Get_Quaternion()[1];
//     float q2 = BSP_BMI088.Get_Quaternion()[2];
//     float q3 = BSP_BMI088.Get_Quaternion()[3];
//     float temperature = BSP_BMI088.BMI088_Accel.Get_Now_Temperature();
//     float calculating_time = BSP_BMI088.Get_Calculating_Time();
//     float loss = BSP_BMI088.Get_Accel_Chi_Square_Loss();
//     float origin_accel_x = BSP_BMI088.Get_Original_Accel()[0][0];
//     float origin_accel_y = BSP_BMI088.Get_Original_Accel()[1][0];
//     float origin_accel_z = BSP_BMI088.Get_Original_Accel()[2][0];
//     float origin_gyro_x = BSP_BMI088.Get_Original_Gyro()[0][0];
//     float origin_gyro_y = BSP_BMI088.Get_Original_Gyro()[1][0];
//     float origin_gyro_z = BSP_BMI088.Get_Original_Gyro()[2][0];
//     float now_time = SYS_Timestamp.Get_Now_Microsecond() / 1000000.0f;
//     float accel_x = BSP_BMI088.Get_Accel()[0][0];
//     float accel_y = BSP_BMI088.Get_Accel()[1][0];
//     float accel_z = BSP_BMI088.Get_Accel()[2][0];
//     float gyro_x = BSP_BMI088.Get_Gyro()[0][0];
//     float gyro_y = BSP_BMI088.Get_Gyro()[1][0];
//     float gyro_z = BSP_BMI088.Get_Gyro()[2][0];
//     float rotation_matrix_r00 = BSP_BMI088.Get_Rotation_Matrix()[0][0];
//     float rotation_matrix_r01 = BSP_BMI088.Get_Rotation_Matrix()[0][1];
//     float rotation_matrix_r02 = BSP_BMI088.Get_Rotation_Matrix()[0][2];
//     float rotation_matrix_r10 = BSP_BMI088.Get_Rotation_Matrix()[1][0];
//     float rotation_matrix_r11 = BSP_BMI088.Get_Rotation_Matrix()[1][1];
//     float rotation_matrix_r12 = BSP_BMI088.Get_Rotation_Matrix()[1][2];
//     float rotation_matrix_r20 = BSP_BMI088.Get_Rotation_Matrix()[2][0];
//     float rotation_matrix_r21 = BSP_BMI088.Get_Rotation_Matrix()[2][1];
//     float rotation_matrix_r22 = BSP_BMI088.Get_Rotation_Matrix()[2][2];
//     float motor_target_angle = motor.Get_Target_Angle();
//     float motor_now_angle = motor.Get_Now_Angle();
//     float motor_target_omega = motor.Get_Target_Omega();
//     float motor_now_omega = motor.Get_Now_Omega();
//     float motor_target_torque = motor.Get_Target_Torque();
//     float motor_now_torque = motor.Get_Now_Torque();
//     float filter_omega = filter_kalman.Vector_X[1][0];
//
//     // 串口绘图
//     // IMU常规显示
//     Vofa_USB.Set_Data(23, &yaw, &pitch, &roll, &q0, &q1, &q2, &q3, &temperature, &calculating_time, &loss, &origin_accel_x, &origin_accel_y, &origin_accel_z, &origin_gyro_x, &origin_gyro_y, &origin_gyro_z, &now_time, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
//     // Vofa_USB.Set_Data(7, &motor_target_angle, &motor_now_angle, &motor_target_omega, &motor_now_omega, &motor_target_torque, &motor_now_torque, &filter_omega);
//     Vofa_USB.TIM_1ms_Write_PeriodElapsedCallback();
//
//     TIM_1ms_CAN_PeriodElapsedCallback();
//     // 喂狗
//     TIM_1ms_IWDG_PeriodElapsedCallback();
// }
//
// /**
//  * @brief 每125us调用一次
//  *
//  */
// void Task125us_Callback()
// {
//     BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
// }
//
// /**
//  * @brief 每10us调用一次
//  *
//  */
// void Task10us_Callback()
// {
//     BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
// }
//
// /**
//  * @brief 初始化任务
//  *
//  */
// void Task_Init()
// {
//     SYS_Timestamp.Init(&htim5);
//     // 串口绘图的USB
//     USB_Init(Serial_USB_Call_Back);
//     // 陀螺仪的SPI
//     SPI_Init(&hspi2, SPI2_Callback);
//     // WS2812的SPI
//     SPI_Init(&hspi6, nullptr);
//     // 电机的CAN
//     CAN_Init(&hfdcan1, CAN1_Callback);
//     // 电源的ADC
//     ADC_Init(&hadc1, 1);
//     // flash的OSPI
//     OSPI_Init(&hospi2, nullptr, nullptr);
//
//     // 定时器中断初始化
//     HAL_TIM_Base_Start_IT(&htim4);
//     HAL_TIM_Base_Start_IT(&htim5);
//     HAL_TIM_Base_Start_IT(&htim6);
//     HAL_TIM_Base_Start_IT(&htim7);
//     HAL_TIM_Base_Start_IT(&htim8);
//
//     Vofa_USB.Init(4, reinterpret_cast<const char **>(Vofa_Variable_Assignment_List));
//
//     BSP_WS2812.Init(0, 0, 0);
//
//     BSP_Buzzer.Init();
//
//     BSP_Power.Init();
//
//     BSP_Key.Init();
//
//     BSP_BMI088.Init();
//
//     motor.PID_Angle.Init(12.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
//     motor.PID_Omega.Init(0.03f, 5.0f, 0.0f, 0.0f, 0.2f, 0.2f);
//     motor.Init(&hfdcan1, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE, 0, PI / 6);
//     A[0][0] = 1.0f;
//     A[0][1] = 0.001f;
//     A[1][0] = 0.0f;
//     A[1][1] = 1.0f;
//     B[0][0] = 0.0f;
//     B[1][0] = 0.0f;
//     H[0][0] = 1.0f;
//     H[0][1] = 0.0f;
//     H[1][0] = 0.0f;
//     H[1][1] = 1.0f;
//
//     // 调参侠
//     Q[0][0] = 0.001f;
//     Q[0][1] = 0.0f;
//     Q[1][0] = 0.0f;
//     Q[1][1] = 0.1f;
//     R[0][0] = 0.001f;
//     R[0][1] = 0.0f;
//     R[1][0] = 0.0f;
//     R[1][1] = 1.0f;
//     P[0][0] = 1.0f;
//     P[0][1] = 0.0f;
//     P[1][0] = 0.0f;
//     P[1][1] = 1.0f;
//     filter_kalman.Init(A, B, H, Q, R, P);
//
//     BSP_W25Q64JV.Init();
//
//     // 标记初始化完成
//     init_finished = true;
// }
//
// /**
//  * @brief 前台循环任务
//  *
//  */
// void Task_Loop()
// {
//     Namespace_SYS_Timestamp::Delay_Millisecond(1);
// }
//
// /**
//  * @brief GPIO中断回调函数
//  *
//  * @param GPIO_Pin 中断引脚
//  */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (!init_finished)
//     {
//         return;
//     }
//
//     if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
//     {
//         BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
//     }
// }
//
// /**
//  * @brief 定时器中断回调函数
//  *
//  * @param htim
//  */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (!init_finished)
//     {
//         return;
//     }
//
//     // 选择回调函数
//     if (htim->Instance == TIM4)
//     {
//         Task10us_Callback();
//     }
//     else if (htim->Instance == TIM5)
//     {
//         Task3600s_Callback();
//     }
//     else if (htim->Instance == TIM6)
//     {
//         Task1s_Callback();
//     }
//     else if (htim->Instance == TIM7)
//     {
//         Task1ms_Callback();
//     }
//     else if (htim->Instance == TIM8)
//     {
//         Task125us_Callback();
//     }
// }
//
// /************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/


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
    // Vofa_JustFloat.Bind_Channel(0,&chassis_power_cmd_est);
    // Vofa_JustFloat.Bind_Channel(1,&chassis_power_limited_est);
    // Vofa_JustFloat.Bind_Channel(2,&chassis_power_limit_max_dbg);
    // Vofa_JustFloat.Bind_Channel(0,&Chassis_Motor_0_Omega);
    // Vofa_JustFloat.Bind_Channel(1,&Chassis_Motor_1_Omega);
    // Vofa_JustFloat.Bind_Channel(2,&Chassis_Motor_0_Current);
    // Vofa_JustFloat.Bind_Channel(3,&Chassis_Motor_1_Current);
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
    // Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Leg_Wheel[1].PID_Angle);
    // Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Track_Wheel[0].PID_Omega);
    // Vofa_Tuner.Bind_PID(1100,&robot.Chassis.Track_Wheel[1].PID_Omega);
    //Vofa_Tuner.Bind_PID(1000,&robot.Chassis.Track_Wheel[0].PID_Omega);

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

    //DM_IMU.Init(&hfdcan2,0x05,0x15);
    // 定时器初始化
    // TIM_Init(&htim4,Task100us_TIM4_Callback);
    // TIM_Init(&htim5,Task1ms_TIM5_Callback);
//     // 定时器中断初始化
    HAL_TIM_Base_Start(&htim5);
//     HAL_TIM_Base_Start_IT(&htim4);
//     HAL_TIM_Base_Start_IT(&htim5);
//     HAL_TIM_Base_Start_IT(&htim6);
//     HAL_TIM_Base_Start_IT(&htim7);
//     HAL_TIM_Base_Start_IT(&htim8);
    // 设备层初始化

    // 牢雄初始化
    robot.Init();

    // VOFA+串口绘图初始化
    Vofa_JustFloat_Init();
    Vofa_Tuner_Init();

    UART_Init(&huart1,Referee_UART1_Callback,512);           //遥控器
   UART_Init(&huart7,Vofa_Tuner_UART7_Callback,64);                //VOFA

    // 定时器中断初始化
    //HAL_TIM_Base_Start_IT(&htim4);
    // HAL_TIM_Base_Start_IT(&htim5);
    //HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);                  //1ms调度
    //HAL_TIM_Base_Start_IT(&htim8);


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
     // Namespace_SYS_Timestamp::Delay_Millisecond(1);
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
    // if (htim->Instance == TIM7)
    // {
    //     Task1ms_TIM7_Callback();
    // }
}