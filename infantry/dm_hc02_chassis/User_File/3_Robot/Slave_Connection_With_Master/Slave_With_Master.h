//
// Created by Lenovo on 2026/4/7.
//

#ifndef TEST_ROBOWAKER_SLAVE_TO_MASTER_H
#define TEST_ROBOWAKER_SLAVE_TO_MASTER_H

#include "1_Middleware/1_Driver/CAN/drv_can.h"
#include "2_Device/Referee/dvc_referee.h"
#include <stdint.h>
#include <string.h>


// 下板发送到上板ID
enum Enum_Slave_TO_Master_ID
{
    Master_To_Slave_ID = 0x310,
};

// 下板接收上板ID
enum Enum_Master_To_Slave_ID
{
    Slave_To_Master_ID_Chassis = 0x311,
};

// 与上板通信的状态
enum Enum_Communication_Status_With_Master
{
    Communication_Status_DISABLE = 0,
    Communication_Status_ENABLE = 1,
};

// 上板发下来的c,n,s档
enum Enum_Robot_Status
{
    Robot_Status_DISABLE = 0,
    Robot_Status_Remote,
    Robot_Status_Computer
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

    Class_Referee *Referee;

    Struct_Slave_To_Master_Chassis_Data Chassis_Data;
    Struct_Slave_To_Master_Booster_Data Booster_Data;
    Struct_Slave_To_Master_Status_Data Status_Data;

    void Init(FDCAN_HandleTypeDef *hcan);
    void CAN_RxCpltCallback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);
    void TIM_10ms_Send_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    inline uint16_t Get_Chassis_Power_Max();
    inline uint16_t Get_Chassis_Energy_Buffer();
    inline uint16_t Get_Booster_Heat_Max();
    inline uint16_t Get_Booster_Heat_CD();
    inline Enum_Communication_Status_With_Master Get_Communication_Status_With_Master();
    inline Enum_Robot_Status Get_VT13_Robot_Status();
    inline float Get_VT13_Left_X();
    inline float Get_VT13_Left_Y();
    inline float Get_VT13_Right_Y();
    inline float Get_VT13_Dial();
    inline uint8_t Get_VT13_Trig();
    inline uint8_t Get_VT13_Supercap();
    inline uint8_t Get_VT13_Auto_Get_Up_Step();

protected:
    Struct_CAN_Manage_Object *CAN_Manage_Object = nullptr;

    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    uint16_t Chassis_Power_Max = 0;
    uint16_t Chassis_Energy_Buffer = 0;
    uint16_t Booster_Heat_Max = 0;
    uint16_t Booster_Heat_CD = 0;

    uint16_t Last_Valid_Chassis_Power_Max = 70;
    uint32_t Last_Valid_Chassis_Power_Max_Tick = 0;

    Enum_Robot_Status Robot_Status = Robot_Status_DISABLE;
    Enum_Communication_Status_With_Master Communication_Status_S_To_M = Communication_Status_DISABLE;
    int8_t  VT13_Left_X = 0;
    int8_t  VT13_Left_Y = 0;
    int8_t  VT13_Right_Y = 0;
    int8_t  VT13_Dial = 0;
    uint8_t VT13_Trig = 0;
    uint8_t VT13_Supercap = 0;
    uint8_t VT13_Auto_Get_Up_Step = 0;

    bool Data_Rx_Update(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);
    void Referee_Data_Update();
    void Data_Tx_Update();

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



//上板发给下板
inline Enum_Communication_Status_With_Master Class_Slave_With_Master::Get_Communication_Status_With_Master()
{
    return Communication_Status_S_To_M;
}

inline Enum_Robot_Status Class_Slave_With_Master::Get_VT13_Robot_Status()
{
    return Robot_Status;
}

inline float Class_Slave_With_Master::Get_VT13_Left_X()
{
    return (float)(VT13_Left_X*0.01f);
}

inline float Class_Slave_With_Master::Get_VT13_Left_Y()
{
    return (float)(VT13_Left_Y*0.01f);
}

inline float Class_Slave_With_Master::Get_VT13_Right_Y()
{
    return (float)(VT13_Right_Y*0.01f);
}

inline float Class_Slave_With_Master::Get_VT13_Dial()
{
    return (float)(VT13_Dial*0.01f);
}

inline uint8_t Class_Slave_With_Master::Get_VT13_Trig()
{
    return VT13_Trig;
}

inline uint8_t Class_Slave_With_Master::Get_VT13_Supercap()
{
    return VT13_Supercap;
}

inline uint8_t Class_Slave_With_Master::Get_VT13_Auto_Get_Up_Step()
{
    return VT13_Auto_Get_Up_Step;
}

#endif //TEST_ROBOWAKER_SLAVE_TO_MASTER_H
