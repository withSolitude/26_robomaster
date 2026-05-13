//
// Created by Lenovo on 2026/4/7.
//
#include "3_Robot/Slave_Connection_With_Master/Slave_With_Master.h"

void Class_Slave_With_Master::Init(FDCAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN_Manage_Object = &CAN1_Manage_Object;
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN_Manage_Object = &CAN2_Manage_Object;
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN_Manage_Object = &CAN3_Manage_Object;
    }
    else
    {
        CAN_Manage_Object = nullptr;
    }

    memset(&Chassis_Data, 0, sizeof(Chassis_Data));
    memset(&Booster_Data, 0, sizeof(Booster_Data));
    memset(&Status_Data, 0, sizeof(Status_Data));

    Chassis_Power_Max = 0;
    Chassis_Energy_Buffer = 0;
    Booster_Heat_Max = 0;
    Booster_Heat_CD = 0;

    Last_Valid_Chassis_Power_Max = 70;
    Last_Valid_Chassis_Power_Max_Tick = HAL_GetTick();
}

void Class_Slave_With_Master::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Pre_Flag == Flag)
    {
        Communication_Status_S_To_M = Communication_Status_DISABLE;

        VT13_Left_X = 0;
        VT13_Left_Y = 0;
        VT13_Right_Y = 0;
        VT13_Dial = 0;

        VT13_Trig = 0;
        VT13_Supercap = 0;
        VT13_Auto_Get_Up_Step = 0;
    }
    else
    {
        Communication_Status_S_To_M = Communication_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

void Class_Slave_With_Master::TIM_10ms_Send_PeriodElapsedCallback()
{
    if (Referee == NULL || CAN_Manage_Object == NULL || CAN_Manage_Object->CAN_Handler == NULL)
        return;

    Referee_Data_Update();
    Data_Tx_Update();

}

void Class_Slave_With_Master::Referee_Data_Update()
{
    Chassis_Data.Chassis_Energy_Buffer = Referee->Get_Chassis_Energy_Buffer();
    Chassis_Data.Chassis_Power_Max = Referee->Get_Self_Chassis_Power_Max();
    Booster_Data.Booster_Heat_Max = Referee->Get_Self_Booster_Heat_Max();
    Booster_Data.Booster_Heat_CD = Referee->Get_Self_Booster_Heat_CD();
    Booster_Data.Booster_Heat_Now = Referee->Get_Booster_17mm_Heat();
}

void Class_Slave_With_Master::Data_Tx_Update()
{
    uint8_t tx_data[8];

    // tx_data[0] = (uint8_t)(Chassis_Data.Chassis_Power_Max & 0x00ff);
    // tx_data[1] = (uint8_t)((Chassis_Data.Chassis_Power_Max >> 8) & 0x00ff);
    tx_data[0] = (uint8_t)(Booster_Data.Booster_Heat_Max & 0x00ff);
    tx_data[1] = (uint8_t)((Booster_Data.Booster_Heat_Max >> 8) & 0x00ff);
    tx_data[2] = (uint8_t)(Booster_Data.Booster_Heat_CD & 0x00ff);
    tx_data[3] = (uint8_t)((Booster_Data.Booster_Heat_CD >> 8) & 0x00ff);
    tx_data[4] = (uint8_t)(Booster_Data.Booster_Heat_Now & 0x00ff);
    tx_data[5] = (uint8_t)((Booster_Data.Booster_Heat_Now >> 8) & 0x00ff);
    tx_data[6] = (uint8_t)(Chassis_Data.Chassis_Power_Max & 0x00ff);
    tx_data[7] = (uint8_t)((Chassis_Data.Chassis_Power_Max >> 8) & 0x00ff);
    // tx_data[6] = (uint8_t)(Booster_Data.Booster_Heat_Now & 0x00ff);                  // 拨弹数据
    // tx_data[7] = (uint8_t)((Booster_Data.Booster_Heat_Now >> 8) & 0x00ff);
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler,Slave_To_Master_ID_Chassis,tx_data,8);
}


void Class_Slave_With_Master::CAN_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return;
    }

    if (Data_Rx_Update(Header, Buffer))
    {
        Flag++;
    }
}

bool Class_Slave_With_Master::Data_Rx_Update(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return false;
    }

    if (Header.Identifier != Master_To_Slave_ID)
    {
        return false;
    }

    if (Header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return false;
    }

    switch (Header.Identifier)
    {
    case Master_To_Slave_ID:
    {
            Robot_Status = (Enum_Robot_Status)Buffer[0];
            VT13_Left_X = (int8_t)Buffer[1];
            VT13_Left_Y = (int8_t)Buffer[2];
            VT13_Right_Y = (int8_t)Buffer[3];
            VT13_Dial = (int8_t)Buffer[4];
            VT13_Trig = Buffer[5] ? 1 : 0;
            VT13_Supercap = Buffer[6] ? 1 : 0;
            VT13_Auto_Get_Up_Step = Buffer[7] ? 1 : 0;

    }

    default:
        break;
    }

    return true;
}
