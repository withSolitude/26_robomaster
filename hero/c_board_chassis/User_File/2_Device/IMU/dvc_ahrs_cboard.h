//
// Created by Lenovo on 2026/1/16.
//

#ifndef TEST_ROBOWAKER_DVC_AHRS_BOARDC_H
#define TEST_ROBOWAKER_DVC_AHRS_BOARDC_H

#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include <cstdint>

/**
 *@brief BOARDC AHRS状态
 *
 */
enum Enum_AHRS_BOARDC_Status
{
    AHRS_BOARDC_Status_DISABLE = 0,
    AHRS_BOARDC_Status_ENABLE,
    AHRS_BOARDC_Status_ERROR,
};

/**
 *@brief 滤波类型
 *
 */
enum Enum_AHRS_BOARDC_Filter_Type
{
    AHRS_BOARDC_Filter_Type_Complementary = 0,
    AHRS_BOARDC_Filter_Type_Mahony_6,   // gyro + acc
    AHRS_BOARDC_Filter_Type_Mahony_9,   // gyro + acc + mag
};

/**
 *@brief 轴映射与符号（用于把板载坐标系映射到你底盘坐标系）
 *
 * Map: 0/1/2 分别代表源轴 X/Y/Z
 * Sign: +1 或 -1
 */
struct Struct_AHRS_BOARDC_Orientation
{
    uint8_t Map_Gyro[3] = {0, 1, 2};
    int8_t  Sign_Gyro[3] = {1, 1, 1};

    uint8_t Map_Acc[3] = {0, 1, 2};
    int8_t  Sign_Acc[3] = {1, 1, 1};

    uint8_t Map_Mag[3] = {0, 1, 2};
    int8_t  Sign_Mag[3] = {1, 1, 1};
};

/**
 *@brief 磁力计标定参数（Hard-iron + 简单Soft-iron缩放）
 *
 * 先只用 Offset 就能大幅提升 yaw 稳定性；Scale 默认 1 即可
 */
struct Struct_AHRS_BOARDC_Mag_Calib
{
    float Offset[3] = {0.0f, 0.0f, 0.0f};
    float Scale[3]  = {1.0f, 1.0f, 1.0f};
};

/**
 *@brief 原始数据
 *
 */
struct Struct_AHRS_BOARDC_Raw
{
    int16_t Gyro[3] = {0, 0, 0};
    int16_t Acc[3]  = {0, 0, 0};
    int16_t Mag[3]  = {0, 0, 0};
    int16_t Temp    = 0;
};

/**
 *@brief 输出数据
 *
 */
struct Struct_AHRS_BOARDC_Data
{
    // rad/s
    float Omega[3] = {0.0f, 0.0f, 0.0f};
    // m/s^2
    float Acc[3]   = {0.0f, 0.0f, 0.0f};
    // 磁力计（内部归一化前会做偏置/缩放；输出这里给“校准后原始量”，单位不强依赖）
    float Mag[3]   = {0.0f, 0.0f, 0.0f};

    // Quaternion
    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    // Euler (rad)
    float Roll  = 0.0f;
    float Pitch = 0.0f;
    float Yaw   = 0.0f;

    // Temperature
    float Temperature = 0.0f;
};

class Class_AHRS_BOARDC
{
public:
    void Init(SPI_HandleTypeDef *hspi,
              GPIO_TypeDef *acc_cs_gpiox, uint16_t acc_cs_pin,
              GPIO_TypeDef *gyro_cs_gpiox, uint16_t gyro_cs_pin,
              I2C_HandleTypeDef *hi2c,
              GPIO_TypeDef *mag_rst_gpiox, uint16_t mag_rst_pin,
              Enum_AHRS_BOARDC_Filter_Type filter_type,
              float dt);

    void TIM_1ms_Calculate_PeriodElapsedCallback();
    void TIM_100ms_Alive_PeriodElapsedCallback();

    void Calibrate_Gyro(uint16_t sample_cnt);

    inline void Set_Filter_Type(Enum_AHRS_BOARDC_Filter_Type type);
    inline void Set_Orientation(Struct_AHRS_BOARDC_Orientation ori);
    inline void Set_Mag_Calib(Struct_AHRS_BOARDC_Mag_Calib calib);

    // Mahony 参数
    inline void Set_Mahony_Gain(float kp, float ki);
    // 互补参数
    inline void Set_Complementary_Alpha(float alpha);

    inline Enum_AHRS_BOARDC_Status Get_Status();

    inline float Get_Omega_X();
    inline float Get_Omega_Y();
    inline float Get_Omega_Z();

    inline float Get_Acc_X();
    inline float Get_Acc_Y();
    inline float Get_Acc_Z();

    inline float Get_Mag_X();
    inline float Get_Mag_Y();
    inline float Get_Mag_Z();

    inline float Get_Angle_Roll();
    inline float Get_Angle_Pitch();
    inline float Get_Angle_Yaw();

    inline float Get_Q0();
    inline float Get_Q1();
    inline float Get_Q2();
    inline float Get_Q3();

protected:
    // handles
    SPI_HandleTypeDef *SPI = nullptr;
    I2C_HandleTypeDef *I2C = nullptr;

    // cs pins
    GPIO_TypeDef *ACC_CS_GPIOx = nullptr;
    uint16_t ACC_CS_Pin = 0;

    GPIO_TypeDef *GYRO_CS_GPIOx = nullptr;
    uint16_t GYRO_CS_Pin = 0;

    // mag rst pin
    GPIO_TypeDef *MAG_RST_GPIOx = nullptr;
    uint16_t MAG_RST_Pin = 0;

    // status
    Enum_AHRS_BOARDC_Status Status = AHRS_BOARDC_Status_DISABLE;
    uint32_t Flag = 0;
    uint32_t Pre_Flag = 0;

    // config
    Enum_AHRS_BOARDC_Filter_Type Filter_Type = AHRS_BOARDC_Filter_Type_Mahony_9;
    float Dt = 0.001f;

    Struct_AHRS_BOARDC_Orientation Orientation;
    Struct_AHRS_BOARDC_Mag_Calib Mag_Calib;

    // raw & data
    Struct_AHRS_BOARDC_Raw Raw;
    Struct_AHRS_BOARDC_Data Data;

    // gyro bias (raw)
    float Gyro_Bias_Raw[3] = {0.0f, 0.0f, 0.0f};

    // scaling (BMI088: gyro ±2000dps, acc ±6g)
    const float Gyro_Raw_To_Rad = (1.0f / 16.384f) * (PI / 180.0f);
    const float Acc_Raw_To_Ms2  = (6.0f / 32768.0f) * 9.80665f;

    // complementary
    float Complementary_Alpha = 0.98f;

    // Mahony
    float Mahony_Kp = 3.0f;
    float Mahony_Ki = 0.0f;
    float Mahony_Integral[3] = {0.0f, 0.0f, 0.0f};

    // mag schedule
    uint16_t Mag_Divider_Cnt = 0;
    const uint16_t Mag_Divider = 10; // 10ms 读一次磁力计（1ms回调里每10次做一次I2C）

    // internal init
    bool _BMI088_Init();
    bool _IST8310_Init();

    // internal read
    bool _BMI088_Read_Gyro(int16_t &gx, int16_t &gy, int16_t &gz);
    bool _BMI088_Read_Acc(int16_t &ax, int16_t &ay, int16_t &az);
    bool _IST8310_Read_Mag(int16_t &mx, int16_t &my, int16_t &mz);

    // spi helpers
    uint8_t _BMI088_Read_Reg(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t reg);
    void _BMI088_Write_Reg(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t reg, uint8_t data);
    void _BMI088_Read_Multi(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t start_reg, uint8_t *buf, uint8_t len);

    // filter update
    void _Update_Euler_From_Quaternion();
    void _Mahony6_Update(float gx, float gy, float gz, float ax, float ay, float az);
    void _Mahony9_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void _Complementary_Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

    // utils
    void _Apply_Orientation(float src[3], const uint8_t map[3], const int8_t sign[3], float out[3]);
    void _Wrap_Angle_Rad(float &angle);
    void _Quaternion_Normalize();
};

// ----------------- inline -----------------

void Class_AHRS_BOARDC::Set_Filter_Type(Enum_AHRS_BOARDC_Filter_Type type)
{
    Filter_Type = type;
}

void Class_AHRS_BOARDC::Set_Orientation(Struct_AHRS_BOARDC_Orientation ori)
{
    Orientation = ori;
}

void Class_AHRS_BOARDC::Set_Mag_Calib(Struct_AHRS_BOARDC_Mag_Calib calib)
{
    Mag_Calib = calib;
}

void Class_AHRS_BOARDC::Set_Mahony_Gain(float kp, float ki)
{
    Mahony_Kp = kp;
    Mahony_Ki = ki;
}

void Class_AHRS_BOARDC::Set_Complementary_Alpha(float alpha)
{
    Complementary_Alpha = alpha;
}

Enum_AHRS_BOARDC_Status Class_AHRS_BOARDC::Get_Status()
{
    return Status;
}

float Class_AHRS_BOARDC::Get_Omega_X(){ return Data.Omega[0]; }
float Class_AHRS_BOARDC::Get_Omega_Y(){ return Data.Omega[1]; }
float Class_AHRS_BOARDC::Get_Omega_Z(){ return Data.Omega[2]; }

float Class_AHRS_BOARDC::Get_Acc_X(){ return Data.Acc[0]; }
float Class_AHRS_BOARDC::Get_Acc_Y(){ return Data.Acc[1]; }
float Class_AHRS_BOARDC::Get_Acc_Z(){ return Data.Acc[2]; }

float Class_AHRS_BOARDC::Get_Mag_X(){ return Data.Mag[0]; }
float Class_AHRS_BOARDC::Get_Mag_Y(){ return Data.Mag[1]; }
float Class_AHRS_BOARDC::Get_Mag_Z(){ return Data.Mag[2]; }

float Class_AHRS_BOARDC::Get_Angle_Roll(){ return Data.Roll; }
float Class_AHRS_BOARDC::Get_Angle_Pitch(){ return Data.Pitch; }
float Class_AHRS_BOARDC::Get_Angle_Yaw(){ return Data.Yaw; }

float Class_AHRS_BOARDC::Get_Q0(){ return Data.q0; }
float Class_AHRS_BOARDC::Get_Q1(){ return Data.q1; }
float Class_AHRS_BOARDC::Get_Q2(){ return Data.q2; }
float Class_AHRS_BOARDC::Get_Q3(){ return Data.q3; }

#endif //TEST_ROBOWAKER_DVC_AHRS_BOARDC_H
