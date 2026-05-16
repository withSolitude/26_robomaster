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

void Class_Slave_With_Master::CAN_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Buffer == nullptr)
    {
        return;
    }

    Data_Rx_Update(Header, Buffer);
}

void Class_Slave_With_Master::Data_Rx_Update(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    switch (Header.Identifier)
    {
    case Slave_To_Master_ID_Chassis:
    {
        uint16_t raw_chassis_power_max =
                (uint16_t)(Buffer[0] | (Buffer[1] << 8));
        uint16_t raw_chassis_energy_buffer =
                (uint16_t)(Buffer[2] | (Buffer[3] << 8));

        // 只接受合法功率档位，非法值直接丢弃
        if (Is_Valid_Chassis_Power_Max(raw_chassis_power_max))
        {
            Chassis_Power_Max = raw_chassis_power_max;
            Last_Valid_Chassis_Power_Max = raw_chassis_power_max;
            Last_Valid_Chassis_Power_Max_Tick = HAL_GetTick();
        }
        else
        {
            // 300ms 内没收到合法档位，就回退安全值
            if ((HAL_GetTick() - Last_Valid_Chassis_Power_Max_Tick) > 300U)
            {
                Chassis_Power_Max = 70;
            }
            else
            {
                Chassis_Power_Max = Last_Valid_Chassis_Power_Max;
            }
        }

        // 缓冲能量一般不会上万，顺手做个上限过滤
        if (raw_chassis_energy_buffer < 1000U)
        {
            Chassis_Energy_Buffer = raw_chassis_energy_buffer;
        }
        break;
    }

    case Slave_To_Master_ID_Shoot:
    {
        uint16_t raw_booster_heat_max =
                (uint16_t)(Buffer[0] | (Buffer[1] << 8));
        uint16_t raw_booster_heat_cd =
                (uint16_t)(Buffer[2] | (Buffer[3] << 8));

        if (raw_booster_heat_max < 1000U)
        {
            Booster_Heat_Max = raw_booster_heat_max;
        }

        if (raw_booster_heat_cd < 1000U)
        {
            Booster_Heat_CD = raw_booster_heat_cd;
        }

            Booster_Heat_Now = (uint16_t)(Buffer[4] | (Buffer[5] << 8));
        break;
    }

    case Slave_To_Master_ID_Robot_Status:
        break;

    default:
        break;
    }
}
