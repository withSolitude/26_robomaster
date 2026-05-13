//
// Created by Lenovo on 2026/3/2.
//
#include "2_Device/DM_IMU/dvc_dm_imu.h"

#include "1_Middleware/1_Driver//CAN/drv_can.h"


/**
************************************************************************
* @brief:
float_to_uint: 浮点数转换为无符号整数函数
* @param[in]: x_float:
* @param[in]: x_min:
* @param[in]: x_max:
* @param[in]: bits:
* @retval:
待转换的浮点数
范围最小值
范围最大值
目标无符号整数的位数
无符号整数结果
* @details:
将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个
指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max- x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:
uint_to_float: 无符号整数转换为浮点数函数
* @param[in]: x_int: 待转换的无符号整数
* @param[in]: x_min: 范围最小值
* @param[in]: x_max: 范围最大值
* @param[in]: bits: 无符号整数的位数
* @retval:
浮点数结果
* @details:
果为一个浮点数
将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max- x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}



void Class_DM_IMU::Init(FDCAN_HandleTypeDef *hcan, uint16_t __CAN_ID, uint16_t __MST_ID)
{
    // 绑定can
    if (hcan == nullptr) return;

    if (hcan->Instance == FDCAN1)
        CAN_Manage_Object  = &CAN1_Manage_Object;
    else if (hcan->Instance == FDCAN2)
        CAN_Manage_Object  = &CAN2_Manage_Object;
    else if (hcan->Instance == FDCAN3)
        CAN_Manage_Object  = &CAN3_Manage_Object;
    else
        CAN_Manage_Object  = nullptr;

    imu_status = Enum_DM_IMU_Status_DISABLE;
    CAN_ID   = __CAN_ID;   // 配置帧目标ID（CANID）
    CAN_Rx_ID = __MST_ID;  // 数据帧过滤ID（MSTID）

    Last_Rx_Tick = 0;
    Temperature = 0.0f;
    Acc_X = Acc_Y = Acc_Z = 0.0f;
    Gyro_X = Gyro_Y = Gyro_Z = 0.0f;
    Pitch_Deg = Yaw_Deg = Roll_Deg = 0.0f;
    Quart[0] = Quart[1] = Quart[2] = Quart[3] = 0.0f;
    Last_ID = 0;
    memset(Last_Raw,0,sizeof(Last_Raw));
}

void Class_DM_IMU::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Last_Rx_Tick == Rx_Tick )
        imu_status = Enum_DM_IMU_Status_DISABLE;
    else
        imu_status = Enum_DM_IMU_Status_ENABLE;
    Last_Rx_Tick = Rx_Tick;
}

void Class_DM_IMU::CAN_RxCpltCallback(const uint8_t *Rx_Data)
{
    (void)Rx_Data;

    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr)  return;


    const uint16_t id = (uint16_t)CAN_Manage_Object->Rx_Header.Identifier;
    const uint8_t *d = Rx_Data;

    // 过滤stdId
    if (id != CAN_Rx_ID) return;
    // 现在是：if (id != CAN_Rx_ID) return;
    // if (CAN_Rx_ID != 0 && id != CAN_Rx_ID) return;

    // Last_ID = id;
    // memcpy(Last_Raw,d,8);
    //
    //
    // // 只有 01/02/03/04 才是传感器数据帧
    // Data_Process(d);

    Last_ID = id;
    memcpy(Last_Raw, d, 8);

    Data_Process(d);

    // 收到合法数据帧后，认为本次请求完成
    switch (d[0])
    {
    case 1:
        if (Pending_Request == Enum_DM_IMU_Request_Type_ACCEL)
        {
            Pending_Request = Enum_DM_IMU_Request_Type_NONE;
            Wait_Tick = 0;
        }
        break;

    case 2:
        if (Pending_Request == Enum_DM_IMU_Request_Type_GYRO)
        {
            Pending_Request = Enum_DM_IMU_Request_Type_NONE;
            Wait_Tick = 0;
        }
        break;

    case 3:
        if (Pending_Request == Enum_DM_IMU_Request_Type_EULER)
        {
            Pending_Request = Enum_DM_IMU_Request_Type_NONE;
            Wait_Tick = 0;
        }
        break;

    default:
        break;
    }
}

static inline void DM_IMU_SendRead(Class_DM_IMU *imu, uint8_t rid)
{
    if (imu == nullptr) return;
    // 你类里有 CAN_Manage_Object
    // 这里直接用它发
}

static inline void pack_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

void Class_DM_IMU::Request_Accel()
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;

    uint8_t tx[8] = {0xCC, 0x01, 0x00, 0xDD, 0,0,0,0}; // RID=0x01, CMD=读(0)
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, tx, 8);
}

void Class_DM_IMU::Request_Gyro()
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;

    uint8_t tx[8] = {0xCC, 0x02, 0x00, 0xDD, 0,0,0,0}; // RID=0x02
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, tx, 8);
}

void Class_DM_IMU::Request_Euler()
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;

    uint8_t tx[8] = {0xCC, 0x03, 0x00, 0xDD, 0,0,0,0}; // RID=0x03
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, tx, 8);
}

// 发送寄存器帧：{0xCC, reg, rw, 0xDD, data[4]}
void Class_DM_IMU::Write_Reg(uint8_t reg, uint32_t data)
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;  // 没设置CANID就不发

    uint8_t tx[8] = {0xCC, reg, 1, 0xDD, 0, 0, 0, 0}; // rw=1 写

    // 小端写入 data
    tx[4] = (uint8_t)(data & 0xFF);
    tx[5] = (uint8_t)((data >> 8) & 0xFF);
    tx[6] = (uint8_t)((data >> 16) & 0xFF);
    tx[7] = (uint8_t)((data >> 24) & 0xFF);

    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, tx, 8);
}

void Class_DM_IMU::Read_Reg(uint8_t reg)
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;

    uint8_t tx[8] = {0xCC, reg, 0, 0xDD, 0, 0, 0, 0}; // rw=0 读
    CAN_Transmit_Data(CAN_Manage_Object->CAN_Handler, CAN_ID, tx, 8);
}

// period_ms：主动发送周期（ms），建议 10
// save：是否保存参数（建议true）
void Class_DM_IMU::Enable_Active_Output(uint16_t period_ms, bool save)
{
    if (period_ms == 0) period_ms = 10; // 避免0导致不输出

    // 按达妙例程/手册常见寄存器：
    // 0x09 通信方式（2=CAN）
    // 0x0A 主动发送间隔（ms）
    // 0x0B 主动/请求模式（1=主动）
    // 0xFE 保存参数
    Write_Reg(0x09, 2);
    Write_Reg(0x0A, (uint32_t)period_ms);
    Write_Reg(0x0B, 1);

    if (save)
        Write_Reg(0xFE, 0);
}


void Class_DM_IMU::UpdateTemperature(const uint8_t* pData)
{
    Temperature = (float)pData[1];
}

void Class_DM_IMU::UpdateAccel(const uint8_t* pData)
{
    uint16_t accel[3];

    accel[0] = pData[3]<<8|pData[2];
    accel[1] = pData[5]<<8|pData[4];
    accel[2] = pData[7]<<8|pData[6];

    Acc_X = uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
    Acc_Y = uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
    Acc_Z = uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
}

void Class_DM_IMU::UpdateGyro(const uint8_t* pData)
{
    uint16_t gyro[3];

    gyro[0] = pData[3]<<8|pData[2];
    gyro[1] = pData[5]<<8|pData[4];
    gyro[2] = pData[7]<<8|pData[6];

    Gyro_X = uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
    Gyro_Y = uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
    Gyro_Z = uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}

void Class_DM_IMU::UpdateEuler(const uint8_t* pData)
{
    int euler[3];

    euler[0] = pData[3]<<8|pData[2];
    euler[1] = pData[5]<<8|pData[4];
    euler[2] = pData[7]<<8|pData[6];

    Pitch_Deg = uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
    Yaw_Deg = uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
    Roll_Deg = uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}

void Class_DM_IMU::UpdateQuaternion(const uint8_t* pData)
{
    int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
    int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
    int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
    int z = (pData[6]&0x3F)<<8|pData[7];

    Quart[0] = uint_to_float(w,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    Quart[1] = uint_to_float(x,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    Quart[2] = uint_to_float(y,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
    Quart[3] = uint_to_float(z,Quaternion_CAN_MIN,Quaternion_CAN_MAX,14);
}



void Class_DM_IMU::Data_Process(const uint8_t *pData)
{
    Rx_Tick++;
    switch (pData[0])
    {
        case 1:
            UpdateTemperature(pData);
            UpdateAccel(pData);
            break;
        case 2:
            UpdateGyro(pData);
            break;
        case 3:
            UpdateEuler(pData);
            break;
        case 4:
            UpdateQuaternion(pData);
            break;
    }
}

void Class_DM_IMU::Disable_Active_Output(bool save)
{
    // 按你现有注释：0x0B = 1 表示主动发送
    // 这里按 0 表示请求模式/非主动模式来处理
    Write_Reg(0x09, 2);   // CAN
    Write_Reg(0x0A, 0);   // 主动发送周期清 0
    Write_Reg(0x0B, 0);   // 关闭主动发送，改为请求式

    if (save)
        Write_Reg(0xFE, 0);
}
void Class_DM_IMU::Send_Request(Enum_DM_IMU_Request_Type req)
{
    switch (req)
    {
    case Enum_DM_IMU_Request_Type_ACCEL:
        Request_Accel();
        break;

    case Enum_DM_IMU_Request_Type_GYRO:
        Request_Gyro();
        break;

    case Enum_DM_IMU_Request_Type_EULER:
        Request_Euler();
        break;

    default:
        return;
    }

    Pending_Request = req;
    Wait_Tick = 0;
}
void Class_DM_IMU::TIM_1ms_Poll_PeriodElapsedCallback()
{
    if (CAN_Manage_Object == nullptr || CAN_Manage_Object->CAN_Handler == nullptr) return;
    if (CAN_ID == 0) return;

    // 还在等上一帧回复：超时重发
    if (Pending_Request != Enum_DM_IMU_Request_Type_NONE)
    {
        Wait_Tick++;

        if (Wait_Tick >= Wait_Timeout_ms)
        {
            Send_Request(Pending_Request);   // 重发同一请求
        }
        return;
    }

    // 没有挂起请求，按周期轮询
    Poll_Tick++;
    if (Poll_Tick < Poll_Interval_ms) return;
    Poll_Tick = 0;

    switch (Poll_Index)
    {
    case 0:
        Send_Request(Enum_DM_IMU_Request_Type_ACCEL);
        Poll_Index = 1;
        break;

    case 1:
        Send_Request(Enum_DM_IMU_Request_Type_GYRO);
        Poll_Index = 2;
        break;

    case 2:
    default:
        Send_Request(Enum_DM_IMU_Request_Type_EULER);
        Poll_Index = 0;
        break;
    }
}