//
// Created by Lenovo on 2026/4/7.
//
#include "3_Robot/Slave_Connection_With_Master/Slave_With_Master.h"


/**
 * @brief 初始化
 *
 */
void Class_Slave_With_Master::Init(Class_Referee *__Referee,CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == CAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }

    Referee = __Referee;
    memset(&Chassis_Data, 0, sizeof(Chassis_Data));
    memset(&Booster_Data, 0, sizeof(Booster_Data));
    memset(&Status_Data, 0, sizeof(Status_Data));
    Count = 0;
}

/**
 *@brief 10ms发送
 *
 */
void Class_Slave_With_Master::TIM_10ms_Send_PeriodElapsedCallback()
{
    if (Referee == NULL || CAN_Manage_Object == NULL || CAN_Manage_Object->CAN_Handler == NULL)
        return;

    Referee_Data_Update();
    Send_Chassis_Data();
    Sent_Booster_Data();
    Sent_Status_Data();

    Count++;
}


/**
 *@brief 数据处理
 *
 */
void Class_Slave_With_Master::Referee_Data_Update()
{
    Chassis_Data.Chassis_Energy_Buffer = Referee->Get_Chassis_Energy_Buffer();
    Chassis_Data.Chassis_Power_Max = Referee->Get_Self_Chassis_Power_Max();
    Booster_Data.Booster_Heat_Max = Referee->Get_Self_Booster_Heat_Max();
    Booster_Data.Booster_Heat_CD = Referee->Get_Self_Booster_Heat_CD();
    Booster_Data.Booster_Heat_Now = Referee->Get_Booster_42mm_Heat();
}


/**
 *@brief 发送底盘shuju
 *
 */
void Class_Slave_With_Master::Send_Chassis_Data()
{
    uint8_t tx_data[8];

    tx_data[0] = (uint8_t)(Chassis_Data.Chassis_Power_Max & 0x00ff);
    tx_data[1] = (uint8_t)((Chassis_Data.Chassis_Power_Max >> 8) & 0x00ff);
    tx_data[2] = (uint8_t)(Chassis_Data.Chassis_Energy_Buffer & 0x00ff);
    tx_data[3] = (uint8_t)((Chassis_Data.Chassis_Energy_Buffer >> 8) & 0x00ff);
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    CAN_Send_Data(CAN_Manage_Object->CAN_Handler,Slave_To_Master_ID_Chassis, tx_data, 8);
}


/**
 *@brief
 *
 */
void Class_Slave_With_Master::Sent_Booster_Data()
{
    uint8_t tx_data[8];

    tx_data[0] = (uint8_t)(Booster_Data.Booster_Heat_Max & 0x00ff);
    tx_data[1] = (uint8_t)((Booster_Data.Booster_Heat_Max >> 8) & 0x00ff);
    tx_data[2] = (uint8_t)(Booster_Data.Booster_Heat_CD & 0x00ff);
    tx_data[3] = (uint8_t)((Booster_Data.Booster_Heat_CD >> 8) & 0x00ff);
    tx_data[4] = (uint8_t)(Booster_Data.Booster_Heat_Now & 0x00ff);
    tx_data[5] = (uint8_t)((Booster_Data.Booster_Heat_Now >> 8) & 0x00ff);
    tx_data[6] = 0;
    tx_data[7] = 0;

    CAN_Send_Data(CAN_Manage_Object->CAN_Handler, Slave_To_Master_ID_Shoot, tx_data, 8);
}

/**
 *@brief
 *
 */
void Class_Slave_With_Master::Sent_Status_Data()
{

}

/**
 *@brief CAN接收回调
 *
 */
// void Class_Slave_With_Master::CAN_RxCpltCallback()
// {
//     if (CAN_Manage_Object == NULL)  return;
//
//     // 滑动窗口, 判断电机是否在线
//     Count += 1;
//
//     switch (CAN_Manage_Object->Rx_Buffer.Header.StdId)
//     {
//     case():
//
//
//         break;
//     default:
//         break;
//
//     }
//
//     Data_Rx_Update();
// }

/**
 * @brief 解析上板发下来的数据
 */
void Class_Slave_With_Master::Data_Rx_Update()
{

}








