#ifndef TEST_ROBOWAKER_SLAVE_TO_MASTER_H
#define TEST_ROBOWAKER_SLAVE_TO_MASTER_H

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "2_Device/Referee/dvc_referee.h"
#include <stdint.h>
#include <string.h>

// 下板发到上板ID
enum Enum_Slave_To_Master_ID
{
    Slave_To_Master_ID_Chassis = 0x310,
    Slave_To_Master_ID_Shoot = 0x311,
    Slave_To_Master_ID_Robot_Status = 0x312
};

// 下板发给上板的底盘数据
struct Struct_Slave_To_Master_Chassis_Data
{
    uint16_t Chassis_Power_Max;
    uint16_t Chassis_Energy_Buffer;
    uint8_t Referee_Online;
    uint8_t Referee_Trust;
    uint8_t PM01_Chassis_Status;
    uint8_t Seq;
};

// 下板发给上板：发射裁判数据
struct Struct_Slave_To_Master_Booster_Data
{
    uint16_t Booster_Heat_Max;
    uint16_t Booster_Heat_CD;
    uint16_t Booster_Heat_Now;
    uint8_t PM01_Booster_Status;
    uint8_t Seq;
};

// 下板发给上板：身份/等级/云台状态
struct Struct_Slave_To_Master_Status_Data
{
    uint8_t Self_ID;
    uint8_t Self_Level;
    uint8_t PM01_Gimbal_Status;
    uint8_t Reserved0;
    uint8_t Reserved1;
    uint8_t Reserved2;
    uint8_t Reserved3;
    uint8_t Seq;
};

class Class_Slave_With_Master
{
public:
    Struct_Slave_To_Master_Chassis_Data Chassis_Data;
    Struct_Slave_To_Master_Booster_Data Booster_Data;
    Struct_Slave_To_Master_Status_Data Status_Data;

    void Init(FDCAN_HandleTypeDef *hcan);
    void CAN_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

    inline uint16_t Get_Chassis_Power_Max();
    inline uint16_t Get_Chassis_Energy_Buffer();
    inline uint16_t Get_Booster_Heat_Max();
    inline uint16_t Get_Booster_Heat_CD();
    inline uint16_t Get_Booster_Heat_Now();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;

    uint16_t Chassis_Power_Max = 0;
    uint16_t Chassis_Energy_Buffer = 0;
    uint16_t Booster_Heat_Max = 0;
    uint16_t Booster_Heat_CD = 0;
    uint16_t Booster_Heat_Now = 0;

    uint16_t Last_Valid_Chassis_Power_Max = 70;
    uint32_t Last_Valid_Chassis_Power_Max_Tick = 0;

    static inline uint8_t Is_Valid_Chassis_Power_Max(uint16_t power)
    {
        switch (power)
        {
            case 70:
            case 75:
            case 80:
            case 85:
            case 90:
            case 95:
            case 100:
            case 105:
            case 110:
            case 115:
            case 120:
                return 1;
            default:
                return 0;
        }
    }

    void Data_Rx_Update(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);
};

inline uint16_t Class_Slave_With_Master::Get_Chassis_Power_Max()
{
    return Chassis_Power_Max;
}

inline uint16_t Class_Slave_With_Master::Get_Chassis_Energy_Buffer()
{
    return Chassis_Energy_Buffer;
}

inline uint16_t Class_Slave_With_Master::Get_Booster_Heat_Max()
{
    return Booster_Heat_Max;
}

inline uint16_t Class_Slave_With_Master::Get_Booster_Heat_CD()
{
    return Booster_Heat_CD;
}

inline uint16_t Class_Slave_With_Master::Get_Booster_Heat_Now()
{
    return Booster_Heat_Now;
}
#endif //TEST_ROBOWAKER_SLAVE_TO_MASTER_H
