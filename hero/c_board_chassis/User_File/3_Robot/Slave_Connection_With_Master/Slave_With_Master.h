//
// Created by Lenovo on 2026/4/7.
//

#ifndef TEST_ROBOWAKER_SLAVE_TO_MASTER_H
#define TEST_ROBOWAKER_SLAVE_TO_MASTER_H

#include "1_Driver/CAN/drv_can.h"
#include "2_Device/Referee/dvc_referee.h"

// 下板发到上板ID
enum Enum_Slave_To_Master_ID
{
    Slave_To_Master_ID_Chassis = 0x310,
    Slave_To_Master_ID_Shoot = 0x311,
    Slave_To_Master_ID_Robot_Status = 0x312
};


// // 上板发到下板ID
// enum Enum_Slave_To_Master_ID
// {
//
// };


// 下板发给上板的底盘数据
struct Struct_Slave_To_Master_Chassis_Data
{
    uint16_t Chassis_Power_Max;         // 裁判系统底盘功率上限
    uint16_t Chassis_Energy_Buffer;     // 缓冲能量
    uint8_t Referee_Online;             // 裁判系统在线
    uint8_t Referee_Trust;              // 裁判数据可信
    uint8_t PM01_Chassis_Status;        // 底盘状态
    uint8_t Seq;                        // 帧计数
};

// 下板发给上板：发射裁判数据
struct Struct_Slave_To_Master_Booster_Data
{
    uint16_t Booster_Heat_Max;          // 热量上限
    uint16_t Booster_Heat_CD;           // 冷却值
    uint16_t Booster_Heat_Now;          // 当前热量
    uint8_t PM01_Booster_Status;        // 发射机构状态
    uint8_t Seq;                        // 帧计数
};

// 下板发给上板：身份/等级/云台状态
struct Struct_Slave_To_Master_Status_Data
{
    uint8_t Self_ID;                    // 己方机器人 ID
    uint8_t Self_Level;                 // 等级
    uint8_t PM01_Gimbal_Status;         // 云台状态
    uint8_t Reserved0;
    uint8_t Reserved1;
    uint8_t Reserved2;
    uint8_t Reserved3;
    uint8_t Seq;                        // 帧计数
};

// 上板发给下板的数据
struct Struct_Master_To_Slave_Data
{

};

// 下板
class Class_Slave_With_Master
{
public:

    Class_Referee *Referee;

    // 底盘数据
    Struct_Slave_To_Master_Chassis_Data Chassis_Data;
    // 发射数据
    Struct_Slave_To_Master_Booster_Data Booster_Data;
    // 机器人状态数据
    Struct_Slave_To_Master_Status_Data Status_Data;

    // 上板发给下板
    Struct_Master_To_Slave_Data Master_To_Slave_Data;



    void Init(Class_Referee *__Referee,CAN_HandleTypeDef *hcan);

    void TIM_10ms_Send_PeriodElapsedCallback();

    void CAN_RxCpltCallback();

protected:
    // 初始化相关常量

    //
    uint8_t Count;


    // 绑定的CAN
    Struct_CAN_Manage_Object *CAN_Manage_Object;




    // 常量

    // 内部变量

    // 读变量



    // // 电机状态
    // Enum_Motor_DJI_Status Motor_DJI_Status = Motor_DJI_Status_DISABLE;
    // 电机对外接口信息


    // 写变量

    // 读写变量

    void Referee_Data_Update();

    void Send_Chassis_Data();

    void Sent_Booster_Data();

    void Sent_Status_Data();

    void Data_Rx_Update();

};

#endif //TEST_ROBOWAKER_SLAVE_TO_MASTER_H