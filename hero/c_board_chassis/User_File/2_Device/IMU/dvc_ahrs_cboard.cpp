//
// Created by Lenovo on 2026/1/16.
//

#include "dvc_ahrs_cboard.h"
#include <cmath>

// ---------------- BMI088 registers (common RoboMaster usage) ----------------
// ACC
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_X_LSB        0x12
#define BMI088_ACC_TEMP_MSB     0x22
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_SOFTRESET    0x7E

// GYRO
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_X_LSB       0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BANDWIDTH   0x10
#define BMI088_GYRO_LPM1        0x11
#define BMI088_GYRO_SOFTRESET   0x14

#define BMI088_ACC_CHIP_ID_VAL  0x1E
#define BMI088_GYRO_CHIP_ID_VAL 0x0F
#define BMI088_SOFTRESET_CMD    0xB6

#define BMI088_SPI_TIMEOUT_MS   5

// ---------------- IST8310 registers (typical) ----------------
#define IST8310_I2C_ADDR_7BIT   0x0E
#define IST8310_I2C_ADDR        (IST8310_I2C_ADDR_7BIT << 1)

// common map
#define IST8310_REG_WHO_AM_I    0x00
#define IST8310_REG_STAT1       0x02
#define IST8310_REG_DATA_X_L    0x03
#define IST8310_REG_CTRL1       0x0A
#define IST8310_REG_CTRL2       0x0B
#define IST8310_REG_AVGCNTL     0x41
#define IST8310_REG_PDCNTL      0x42

// some boards whoami=0x10（若你读不到不强依赖，调试时用I2C扫地址更可靠）
#define IST8310_WHO_AM_I_VAL    0x10

static inline void Delay_Us(volatile uint32_t us)
{
    while (us--)
    {
        for (volatile uint32_t i = 0; i < 20; i++) { __NOP(); }
    }
}

void Class_AHRS_BOARDC::Init(SPI_HandleTypeDef *hspi,
                            GPIO_TypeDef *acc_cs_gpiox, uint16_t acc_cs_pin,
                            GPIO_TypeDef *gyro_cs_gpiox, uint16_t gyro_cs_pin,
                            I2C_HandleTypeDef *hi2c,
                            GPIO_TypeDef *mag_rst_gpiox, uint16_t mag_rst_pin,
                            Enum_AHRS_BOARDC_Filter_Type filter_type,
                            float dt)
{
    SPI = hspi;
    I2C = hi2c;

    ACC_CS_GPIOx = acc_cs_gpiox;
    ACC_CS_Pin = acc_cs_pin;

    GYRO_CS_GPIOx = gyro_cs_gpiox;
    GYRO_CS_Pin = gyro_cs_pin;

    MAG_RST_GPIOx = mag_rst_gpiox;
    MAG_RST_Pin = mag_rst_pin;

    Filter_Type = filter_type;
    Dt = dt;

    // CS idle high
    HAL_GPIO_WritePin(ACC_CS_GPIOx, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GYRO_CS_GPIOx, GYRO_CS_Pin, GPIO_PIN_SET);

    // mag rst high（先保持高）
    if (MAG_RST_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(MAG_RST_GPIOx, MAG_RST_Pin, GPIO_PIN_SET);
    }

    bool ok1 = _BMI088_Init();
    bool ok2 = _IST8310_Init();

    if (ok1 == false || ok2 == false)
    {
        Status = AHRS_BOARDC_Status_ERROR;
        return;
    }

    // reset filter state
    Data.q0 = 1.0f; Data.q1 = 0.0f; Data.q2 = 0.0f; Data.q3 = 0.0f;
    Data.Roll = 0.0f; Data.Pitch = 0.0f; Data.Yaw = 0.0f;
    Mahony_Integral[0] = Mahony_Integral[1] = Mahony_Integral[2] = 0.0f;

    Flag = 0;
    Pre_Flag = 0;
    Mag_Divider_Cnt = 0;

    Status = AHRS_BOARDC_Status_ENABLE;
}

bool Class_AHRS_BOARDC::_BMI088_Init()
{
    // 给 ACC CS 一个上升沿（某些板卡上电默认I2C模式，用此动作切SPI更稳）
    HAL_GPIO_WritePin(ACC_CS_GPIOx, ACC_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(ACC_CS_GPIOx, ACC_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    // soft reset
    _BMI088_Write_Reg(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(30);
    _BMI088_Write_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_SOFTRESET, BMI088_SOFTRESET_CMD);
    HAL_Delay(2);

    // check id
    uint8_t gid = _BMI088_Read_Reg(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_CHIP_ID);
    uint8_t aid = _BMI088_Read_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_CHIP_ID);

    if (gid != BMI088_GYRO_CHIP_ID_VAL || aid != BMI088_ACC_CHIP_ID_VAL)
    {
        return false;
    }

    // ACC power
    _BMI088_Write_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_PWR_CONF, 0x00); // active
    HAL_Delay(1);
    _BMI088_Write_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_PWR_CTRL, 0x04); // accel on
    HAL_Delay(50);

    // ACC_CONF: 0xAC => ODR 1600Hz + normal
    _BMI088_Write_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_CONF, 0xAC);
    // ACC_RANGE: 0x01 => ±6g
    _BMI088_Write_Reg(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_RANGE, 0x01);

    // GYRO normal
    _BMI088_Write_Reg(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_LPM1, 0x00);
    HAL_Delay(1);
    // ±2000dps
    _BMI088_Write_Reg(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_RANGE, 0x00);
    // bandwidth / ODR
    _BMI088_Write_Reg(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_BANDWIDTH, 0x02);

    return true;
}

bool Class_AHRS_BOARDC::_IST8310_Init()
{
    if (I2C == nullptr)
    {
        return false;
    }

    // reset pulse (optional but recommended)
    if (MAG_RST_GPIOx != nullptr)
    {
        HAL_GPIO_WritePin(MAG_RST_GPIOx, MAG_RST_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(MAG_RST_GPIOx, MAG_RST_Pin, GPIO_PIN_SET);
        HAL_Delay(10);
    }

    // soft reset
    uint8_t rst = 0x01;
    (void)HAL_I2C_Mem_Write(I2C, IST8310_I2C_ADDR, IST8310_REG_CTRL2, I2C_MEMADD_SIZE_8BIT, &rst, 1, 20);
    HAL_Delay(10);

    // avg / pd (如果某些板子不支持这些寄存器，写失败也不影响后续单次测量读数，调试时可注释)
    uint8_t avg = 0x24;
    (void)HAL_I2C_Mem_Write(I2C, IST8310_I2C_ADDR, IST8310_REG_AVGCNTL, I2C_MEMADD_SIZE_8BIT, &avg, 1, 20);
    uint8_t pd = 0xC0;
    (void)HAL_I2C_Mem_Write(I2C, IST8310_I2C_ADDR, IST8310_REG_PDCNTL, I2C_MEMADD_SIZE_8BIT, &pd, 1, 20);

    // whoami (不作为强依赖)
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(I2C, IST8310_I2C_ADDR, IST8310_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 20) != HAL_OK)
    {
        // I2C 读不到，直接认为失败
        return false;
    }

    // trigger first single measurement
    uint8_t mode = 0x01;
    if (HAL_I2C_Mem_Write(I2C, IST8310_I2C_ADDR, IST8310_REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &mode, 1, 20) != HAL_OK)
    {
        return false;
    }

    return true;
}

void Class_AHRS_BOARDC::Calibrate_Gyro(uint16_t sample_cnt)
{
    if (Status == AHRS_BOARDC_Status_ERROR) return;

    float sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t i = 0; i < sample_cnt; i++)
    {
        int16_t gx, gy, gz;
        if (_BMI088_Read_Gyro(gx, gy, gz))
        {
            sum[0] += (float)gx;
            sum[1] += (float)gy;
            sum[2] += (float)gz;
        }
        HAL_Delay(2);
    }

    Gyro_Bias_Raw[0] = sum[0] / sample_cnt;
    Gyro_Bias_Raw[1] = sum[1] / sample_cnt;
    Gyro_Bias_Raw[2] = sum[2] / sample_cnt;
}

void Class_AHRS_BOARDC::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    if (Status != AHRS_BOARDC_Status_ENABLE) return;

    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    if (_BMI088_Read_Gyro(gx, gy, gz) == false) return;
    if (_BMI088_Read_Acc(ax, ay, az) == false) return;

    Raw.Gyro[0] = gx; Raw.Gyro[1] = gy; Raw.Gyro[2] = gz;
    Raw.Acc[0]  = ax; Raw.Acc[1]  = ay; Raw.Acc[2]  = az;

    // scaling + bias
    float gyro_src[3];
    gyro_src[0] = ((float)gx - Gyro_Bias_Raw[0]) * Gyro_Raw_To_Rad;
    gyro_src[1] = ((float)gy - Gyro_Bias_Raw[1]) * Gyro_Raw_To_Rad;
    gyro_src[2] = ((float)gz - Gyro_Bias_Raw[2]) * Gyro_Raw_To_Rad;

    float acc_src[3];
    acc_src[0] = (float)ax * Acc_Raw_To_Ms2;
    acc_src[1] = (float)ay * Acc_Raw_To_Ms2;
    acc_src[2] = (float)az * Acc_Raw_To_Ms2;

    // apply orientation mapping to your robot frame
    float gyro_mapped[3];
    float acc_mapped[3];
    _Apply_Orientation(gyro_src, Orientation.Map_Gyro, Orientation.Sign_Gyro, gyro_mapped);
    _Apply_Orientation(acc_src,  Orientation.Map_Acc,  Orientation.Sign_Acc,  acc_mapped);

    Data.Omega[0] = gyro_mapped[0];
    Data.Omega[1] = gyro_mapped[1];
    Data.Omega[2] = gyro_mapped[2];

    Data.Acc[0] = acc_mapped[0];
    Data.Acc[1] = acc_mapped[1];
    Data.Acc[2] = acc_mapped[2];

    // -------- mag (every 10ms) --------
    bool mag_ok = false;
    float mag_mapped[3] = {Data.Mag[0], Data.Mag[1], Data.Mag[2]};

    Mag_Divider_Cnt++;
    if (Mag_Divider_Cnt >= Mag_Divider)
    {
        Mag_Divider_Cnt = 0;
        int16_t mx, my, mz;
        if (_IST8310_Read_Mag(mx, my, mz))
        {
            Raw.Mag[0] = mx; Raw.Mag[1] = my; Raw.Mag[2] = mz;

            float mag_src[3];
            mag_src[0] = ((float)mx - Mag_Calib.Offset[0]) * Mag_Calib.Scale[0];
            mag_src[1] = ((float)my - Mag_Calib.Offset[1]) * Mag_Calib.Scale[1];
            mag_src[2] = ((float)mz - Mag_Calib.Offset[2]) * Mag_Calib.Scale[2];

            _Apply_Orientation(mag_src, Orientation.Map_Mag, Orientation.Sign_Mag, mag_mapped);

            Data.Mag[0] = mag_mapped[0];
            Data.Mag[1] = mag_mapped[1];
            Data.Mag[2] = mag_mapped[2];

            mag_ok = true;

            // trigger next single measurement
            uint8_t mode = 0x01;
            (void)HAL_I2C_Mem_Write(I2C, IST8310_I2C_ADDR, IST8310_REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &mode, 1, 10);
        }
    }

    // -------- filters --------
    float gx_r = Data.Omega[0], gy_r = Data.Omega[1], gz_r = Data.Omega[2];
    float ax_r = Data.Acc[0],   ay_r = Data.Acc[1],   az_r = Data.Acc[2];
    float mx_r = Data.Mag[0],   my_r = Data.Mag[1],   mz_r = Data.Mag[2];

    if (Filter_Type == AHRS_BOARDC_Filter_Type_Mahony_6)
    {
        _Mahony6_Update(gx_r, gy_r, gz_r, ax_r, ay_r, az_r);
        _Update_Euler_From_Quaternion();
    }
    else if (Filter_Type == AHRS_BOARDC_Filter_Type_Mahony_9)
    {
        // 如果磁力计暂时还没读到数据，也允许先跑6轴（避免初期卡死）
        if (mag_ok == false && (fabsf(mx_r) + fabsf(my_r) + fabsf(mz_r) < 1e-6f))
        {
            _Mahony6_Update(gx_r, gy_r, gz_r, ax_r, ay_r, az_r);
        }
        else
        {
            _Mahony9_Update(gx_r, gy_r, gz_r, ax_r, ay_r, az_r, mx_r, my_r, mz_r);
        }
        _Update_Euler_From_Quaternion();
    }
    else
    {
        _Complementary_Update(gx_r, gy_r, gz_r, ax_r, ay_r, az_r, mx_r, my_r, mz_r);
    }

    Flag++;
}

void Class_AHRS_BOARDC::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Status == AHRS_BOARDC_Status_ERROR) return;

    if (Flag == Pre_Flag)
    {
        Status = AHRS_BOARDC_Status_DISABLE;
    }
    else
    {
        Pre_Flag = Flag;
        Status = AHRS_BOARDC_Status_ENABLE;
    }
}

// ---------------- SPI helpers ----------------

uint8_t Class_AHRS_BOARDC::_BMI088_Read_Reg(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(reg | 0x80); // read
    tx[1] = 0x00;

    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(SPI, tx, rx, 2, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_SET);

    return rx[1];
}

void Class_AHRS_BOARDC::_BMI088_Write_Reg(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t reg, uint8_t data)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(reg & 0x7F); // write
    tx[1] = data;

    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(SPI, tx, 2, BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_SET);

    Delay_Us(5);
}

void Class_AHRS_BOARDC::_BMI088_Read_Multi(GPIO_TypeDef *cs_gpiox, uint16_t cs_pin, uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    // cmd + dummy + data...
    // 为了兼容 accel/gyro 都按“dummy”读法（更稳）
    uint8_t tx[1 + 1 + 8] = {0};
    uint8_t rx[1 + 1 + 8] = {0};

    tx[0] = (uint8_t)(start_reg | 0x80);

    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(SPI, tx, rx, (uint16_t)(len + 2), BMI088_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(cs_gpiox, cs_pin, GPIO_PIN_SET);

    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = rx[i + 2];
    }
}

// ---------------- device read ----------------

bool Class_AHRS_BOARDC::_BMI088_Read_Gyro(int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t buf[6];
    _BMI088_Read_Multi(GYRO_CS_GPIOx, GYRO_CS_Pin, BMI088_GYRO_X_LSB, buf, 6);

    gx = (int16_t)((buf[1] << 8) | buf[0]);
    gy = (int16_t)((buf[3] << 8) | buf[2]);
    gz = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

bool Class_AHRS_BOARDC::_BMI088_Read_Acc(int16_t &ax, int16_t &ay, int16_t &az)
{
    uint8_t buf[6];
    _BMI088_Read_Multi(ACC_CS_GPIOx, ACC_CS_Pin, BMI088_ACC_X_LSB, buf, 6);

    ax = (int16_t)((buf[1] << 8) | buf[0]);
    ay = (int16_t)((buf[3] << 8) | buf[2]);
    az = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

bool Class_AHRS_BOARDC::_IST8310_Read_Mag(int16_t &mx, int16_t &my, int16_t &mz)
{
    if (I2C == nullptr) return false;

    // 可选：读 STAT1 判断数据就绪（不同版本可能bit位不一样，不强依赖）
    uint8_t stat = 0;
    if (HAL_I2C_Mem_Read(I2C, IST8310_I2C_ADDR, IST8310_REG_STAT1, I2C_MEMADD_SIZE_8BIT, &stat, 1, 10) != HAL_OK)
    {
        return false;
    }

    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(I2C, IST8310_I2C_ADDR, IST8310_REG_DATA_X_L, I2C_MEMADD_SIZE_8BIT, buf, 6, 10) != HAL_OK)
    {
        return false;
    }

    mx = (int16_t)((buf[1] << 8) | buf[0]);
    my = (int16_t)((buf[3] << 8) | buf[2]);
    mz = (int16_t)((buf[5] << 8) | buf[4]);

    return true;
}

// ---------------- math utils ----------------

void Class_AHRS_BOARDC::_Apply_Orientation(float src[3], const uint8_t map[3], const int8_t sign[3], float out[3])
{
    out[0] = (float)sign[0] * src[map[0]];
    out[1] = (float)sign[1] * src[map[1]];
    out[2] = (float)sign[2] * src[map[2]];
}

void Class_AHRS_BOARDC::_Wrap_Angle_Rad(float &angle)
{
    if (angle > PI) angle -= 2.0f * PI;
    else if (angle < -PI) angle += 2.0f * PI;
}

void Class_AHRS_BOARDC::_Quaternion_Normalize()
{
    float n = sqrtf(Data.q0*Data.q0 + Data.q1*Data.q1 + Data.q2*Data.q2 + Data.q3*Data.q3);
    if (n < 1e-6f) return;
    Data.q0 /= n;
    Data.q1 /= n;
    Data.q2 /= n;
    Data.q3 /= n;
}

void Class_AHRS_BOARDC::_Update_Euler_From_Quaternion()
{
    float q0 = Data.q0, q1 = Data.q1, q2 = Data.q2, q3 = Data.q3;

    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
    Data.Roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >= 1.0f) Data.Pitch = PI/2.0f;
    else if (sinp <= -1.0f) Data.Pitch = -PI/2.0f;
    else Data.Pitch = asinf(sinp);

    float siny_cosp = 2.0f * (q0*q3 + q1*q2);
    float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
    Data.Yaw = atan2f(siny_cosp, cosy_cosp);

    _Wrap_Angle_Rad(Data.Roll);
    _Wrap_Angle_Rad(Data.Pitch);
    _Wrap_Angle_Rad(Data.Yaw);
}

// ---------------- Complementary ----------------
// RP: accel 修正，Yaw: gyro积分 + （可选）磁力计倾斜补偿 yaw
void Class_AHRS_BOARDC::_Complementary_Update(float gx, float gy, float gz,
                                             float ax, float ay, float az,
                                             float mx, float my, float mz)
{
    // accel normalize
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-6f)
    {
        Data.Roll  += gx * Dt;
        Data.Pitch += gy * Dt;
        Data.Yaw   += gz * Dt;
        _Wrap_Angle_Rad(Data.Roll);
        _Wrap_Angle_Rad(Data.Pitch);
        _Wrap_Angle_Rad(Data.Yaw);
        return;
    }
    ax /= an; ay /= an; az /= an;

    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    float roll_gyro  = Data.Roll  + gx * Dt;
    float pitch_gyro = Data.Pitch + gy * Dt;
    float yaw_gyro   = Data.Yaw   + gz * Dt;

    Data.Roll  = Complementary_Alpha * roll_gyro  + (1.0f - Complementary_Alpha) * roll_acc;
    Data.Pitch = Complementary_Alpha * pitch_gyro + (1.0f - Complementary_Alpha) * pitch_acc;

    // yaw: default integrate
    Data.Yaw = yaw_gyro;

    // 如果磁力计有效，用 tilt-comp yaw 修正（可选）
    float mn = sqrtf(mx*mx + my*my + mz*mz);
    if (mn > 1e-6f)
    {
        mx /= mn; my /= mn; mz /= mn;

        float cr = cosf(Data.Roll),  sr = sinf(Data.Roll);
        float cp = cosf(Data.Pitch), sp = sinf(Data.Pitch);

        // tilt compensation
        float mxh = mx*cp + my*sp*sr + mz*sp*cr;
        float myh = my*cr - mz*sr;

        float yaw_mag = atan2f(-myh, mxh);

        // 用很小权重修正 yaw（避免抖动）
        float beta = 0.02f; // 可调：0.01~0.05
        float yaw_err = yaw_mag - Data.Yaw;
        // wrap error
        if (yaw_err > PI) yaw_err -= 2.0f*PI;
        else if (yaw_err < -PI) yaw_err += 2.0f*PI;

        Data.Yaw += beta * yaw_err;
    }

    _Wrap_Angle_Rad(Data.Roll);
    _Wrap_Angle_Rad(Data.Pitch);
    _Wrap_Angle_Rad(Data.Yaw);

    // 给 quaternion 同步一下（用欧拉转四元数，互补滤波下足够）
    float cr2 = cosf(Data.Roll*0.5f),  sr2 = sinf(Data.Roll*0.5f);
    float cp2 = cosf(Data.Pitch*0.5f), sp2 = sinf(Data.Pitch*0.5f);
    float cy2 = cosf(Data.Yaw*0.5f),   sy2 = sinf(Data.Yaw*0.5f);

    Data.q0 = cr2*cp2*cy2 + sr2*sp2*sy2;
    Data.q1 = sr2*cp2*cy2 - cr2*sp2*sy2;
    Data.q2 = cr2*sp2*cy2 + sr2*cp2*sy2;
    Data.q3 = cr2*cp2*sy2 - sr2*sp2*cy2;
    _Quaternion_Normalize();
}

// ---------------- Mahony 6-axis ----------------
void Class_AHRS_BOARDC::_Mahony6_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    // normalize acc
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-6f) return;
    ax /= an; ay /= an; az /= an;

    float q0 = Data.q0, q1 = Data.q1, q2 = Data.q2, q3 = Data.q3;

    // estimated gravity
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // error = acc x v
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    if (Mahony_Ki > 0.0f)
    {
        Mahony_Integral[0] += Mahony_Ki * ex * Dt;
        Mahony_Integral[1] += Mahony_Ki * ey * Dt;
        Mahony_Integral[2] += Mahony_Ki * ez * Dt;
    }
    else
    {
        Mahony_Integral[0] = Mahony_Integral[1] = Mahony_Integral[2] = 0.0f;
    }

    gx += Mahony_Kp * ex + Mahony_Integral[0];
    gy += Mahony_Kp * ey + Mahony_Integral[1];
    gz += Mahony_Kp * ez + Mahony_Integral[2];

    float half_dt = 0.5f * Dt;

    Data.q0 += (-q1*gx - q2*gy - q3*gz) * half_dt;
    Data.q1 += ( q0*gx + q2*gz - q3*gy) * half_dt;
    Data.q2 += ( q0*gy - q1*gz + q3*gx) * half_dt;
    Data.q3 += ( q0*gz + q1*gy - q2*gx) * half_dt;

    _Quaternion_Normalize();
}

// ---------------- Mahony 9-axis ----------------
// 典型 Mahony 扩展：在 6轴误差基础上加入磁场参考方向误差
void Class_AHRS_BOARDC::_Mahony9_Update(float gx, float gy, float gz,
                                       float ax, float ay, float az,
                                       float mx, float my, float mz)
{
    // normalize acc
    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-6f) return;
    ax /= an; ay /= an; az /= an;

    // normalize mag
    float mn = sqrtf(mx*mx + my*my + mz*mz);
    if (mn < 1e-6f)
    {
        _Mahony6_Update(gx, gy, gz, ax, ay, az);
        return;
    }
    mx /= mn; my /= mn; mz /= mn;

    float q0 = Data.q0, q1 = Data.q1, q2 = Data.q2, q3 = Data.q3;

    // reference direction of Earth's magnetic field
    // hx, hy from q and m
    float hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3) + 2.0f*my*(q1*q2 - q0*q3) + 2.0f*mz*(q1*q3 + q0*q2);
    float hy = 2.0f*mx*(q1*q2 + q0*q3) + 2.0f*my*(0.5f - q1*q1 - q3*q3) + 2.0f*mz*(q2*q3 - q0*q1);
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = 2.0f*mx*(q1*q3 - q0*q2) + 2.0f*my*(q2*q3 + q0*q1) + 2.0f*mz*(0.5f - q1*q1 - q2*q2);

    // estimated gravity
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // estimated magnetic field
    float wx = 2.0f*bx*(0.5f - q2*q2 - q3*q3) + 2.0f*bz*(q1*q3 - q0*q2);
    float wy = 2.0f*bx*(q1*q2 - q0*q3) + 2.0f*bz*(q0*q1 + q2*q3);
    float wz = 2.0f*bx*(q0*q2 + q1*q3) + 2.0f*bz*(0.5f - q1*q1 - q2*q2);

    // error: gravity + magnetic
    float ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    float ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    float ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    if (Mahony_Ki > 0.0f)
    {
        Mahony_Integral[0] += Mahony_Ki * ex * Dt;
        Mahony_Integral[1] += Mahony_Ki * ey * Dt;
        Mahony_Integral[2] += Mahony_Ki * ez * Dt;
    }
    else
    {
        Mahony_Integral[0] = Mahony_Integral[1] = Mahony_Integral[2] = 0.0f;
    }

    gx += Mahony_Kp * ex + Mahony_Integral[0];
    gy += Mahony_Kp * ey + Mahony_Integral[1];
    gz += Mahony_Kp * ez + Mahony_Integral[2];

    float half_dt = 0.5f * Dt;

    Data.q0 += (-q1*gx - q2*gy - q3*gz) * half_dt;
    Data.q1 += ( q0*gx + q2*gz - q3*gy) * half_dt;
    Data.q2 += ( q0*gy - q1*gz + q3*gx) * half_dt;
    Data.q3 += ( q0*gz + q1*gy - q2*gx) * half_dt;

    _Quaternion_Normalize();
}
