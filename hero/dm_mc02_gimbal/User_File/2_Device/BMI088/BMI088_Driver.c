/**
 * @file BMI088_Driver.c
 * @brief BMI088传感器HAL库驱动实现
 * @author 从CBoard IMU原始代码移植并适配HAL库
 */
#include "BMI088_Driver.h"
#include "BMI088_Reg.h"
#include "math.h"
#include "string.h"

static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, (uint8_t)(0x01U | 0x80U), BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, (uint8_t)(0x02U | 0x80U), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

static void SPI_Write_Reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_data[2];
    tx_data[0] = (uint8_t)(reg_addr & 0x7F);
    tx_data[1] = data;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, tx_data, 2, 100);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

static uint8_t SPI_Read_Accel_Reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr)
{
    uint8_t result = 0;
    uint8_t tx_data[3];
    uint8_t rx_data[3] = {0};
    tx_data[0] = (uint8_t)(reg_addr | 0x80);
    tx_data[1] = 0x55;
    tx_data[2] = 0x55;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 3, 100);
    result = rx_data[2];
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return result;
}

static uint8_t SPI_Read_Gyro_Reg(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t reg_addr)
{
    uint8_t result = 0;
    uint8_t tx_data[2];
    uint8_t rx_data[2] = {0};
    tx_data[0] = (uint8_t)(reg_addr | 0x80);
    tx_data[1] = 0x55;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, 100);
    result = rx_data[1];
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return result;
}

uint8_t BMI088_Accel_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t res = 0;

    res = SPI_Read_Accel_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, ACC_CHIP_ID_ADDR);
    HAL_Delay(1);
    res = SPI_Read_Accel_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, ACC_CHIP_ID_ADDR);
    HAL_Delay(1);

    SPI_Write_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, ACC_SOFTRESET_ADDR, ACC_SOFTRESET_VAL);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    res = SPI_Read_Accel_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, ACC_CHIP_ID_ADDR);
    HAL_Delay(1);
    res = SPI_Read_Accel_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, ACC_CHIP_ID_ADDR);
    HAL_Delay(1);

    if (res != ACC_CHIP_ID_VAL)
        return BMI088_NO_SENSOR;

    for (write_reg_num = 0; write_reg_num < 6; write_reg_num++)
    {
        SPI_Write_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp,
                      write_BMI088_accel_reg_data_error[write_reg_num][0],
                      write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        res = SPI_Read_Accel_Reg(hspi, BMI088_ACC_GPIOx, BMI088_ACC_GPIOp,
                          write_BMI088_accel_reg_data_error[write_reg_num][0]);
        HAL_Delay(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return error == BMI088_NO_ERROR ? BMI088_NO_ERROR : error;
}

uint8_t BMI088_Gyro_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t res = 0;

    res = SPI_Read_Gyro_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GYRO_CHIP_ID_ADDR);
    HAL_Delay(1);
    res = SPI_Read_Gyro_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GYRO_CHIP_ID_ADDR);
    HAL_Delay(1);

    SPI_Write_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GYRO_SOFTRESET_ADDR, GYRO_SOFTRESET_VAL);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    res = SPI_Read_Gyro_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GYRO_CHIP_ID_ADDR);
    HAL_Delay(1);
    res = SPI_Read_Gyro_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GYRO_CHIP_ID_ADDR);
    HAL_Delay(1);

    if (res != GYRO_CHIP_ID_VAL)
        return BMI088_NO_SENSOR;

    for (write_reg_num = 0; write_reg_num < 6; write_reg_num++)
    {
        SPI_Write_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp,
                      write_BMI088_gyro_reg_data_error[write_reg_num][0],
                      write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        res = SPI_Read_Gyro_Reg(hspi, BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp,
                          write_BMI088_gyro_reg_data_error[write_reg_num][0]);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            error |= write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return error == BMI088_NO_ERROR ? BMI088_NO_ERROR : error;
}

uint16_t BMI088_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t accel_error, gyro_error;

    accel_error = BMI088_Accel_Init(hspi);
    gyro_error = BMI088_Gyro_Init(hspi);

    return (accel_error == BMI088_NO_ERROR && gyro_error == BMI088_NO_ERROR) ? BMI088_NO_ERROR : (accel_error | gyro_error);
}

void BMI088_Read_Accel(SPI_HandleTypeDef *hspi, IMU_Data_t *data)
{
    uint8_t tx_buf[8] = {ACC_X_LSB_ADDR | 0x80, 0xFF, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    uint8_t rx_buf[8] = {0};

    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 8, 100);
    HAL_GPIO_WritePin(BMI088_ACC_GPIOx, BMI088_ACC_GPIOp, GPIO_PIN_SET);

    int16_t accel_raw[3];
    accel_raw[0] = (int16_t)(((int16_t)rx_buf[3] << 8) | rx_buf[2]);
    accel_raw[1] = (int16_t)(((int16_t)rx_buf[5] << 8) | rx_buf[4]);
    accel_raw[2] = (int16_t)(((int16_t)rx_buf[7] << 8) | rx_buf[6]);

    data->Accel[0] = (float)accel_raw[0] * BMI088_ACCEL_6G_SEN;
    data->Accel[1] = (float)accel_raw[1] * BMI088_ACCEL_6G_SEN;
    data->Accel[2] = (float)accel_raw[2] * BMI088_ACCEL_6G_SEN;
}

void BMI088_Read_Gyro(SPI_HandleTypeDef *hspi, IMU_Data_t *data)
{
    uint8_t tx_buf[7] = {GYRO_RATE_X_LSB_ADDR | 0x80, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
    uint8_t rx_buf[7] = {0};

    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 7, 100);
    HAL_GPIO_WritePin(BMI088_GYRO_GPIOx, BMI088_GYRO_GPIOp, GPIO_PIN_SET);

    int16_t gyro_raw[3];
    gyro_raw[0] = (int16_t)(((int16_t)rx_buf[2] << 8) | rx_buf[1]);
    gyro_raw[1] = (int16_t)(((int16_t)rx_buf[4] << 8) | rx_buf[3]);
    gyro_raw[2] = (int16_t)(((int16_t)rx_buf[6] << 8) | rx_buf[5]);

    // 始终返回原始数据，不进行零偏补偿
    data->Gyro[0] = (float)gyro_raw[0] * BMI088_GYRO_2000_SEN;
    data->Gyro[1] = (float)gyro_raw[1] * BMI088_GYRO_2000_SEN;
    data->Gyro[2] = (float)gyro_raw[2] * BMI088_GYRO_2000_SEN;
}

void BMI088_Read_IMU(SPI_HandleTypeDef *hspi, IMU_Data_t *data)
{
    BMI088_Read_Accel(hspi, data);
    BMI088_Read_Gyro(hspi, data);
}

void BMI088_Calibrate(IMU_Data_t *data, SPI_HandleTypeDef *hspi, uint16_t cali_times)
{
    float gyro_max[3], gyro_min[3];
    float g_norm_temp, g_norm_max, g_norm_min;
    float gyro_diff[3], g_norm_diff;

    data->AccelScale = 1.0f;

    do
    {
        HAL_Delay(5);
        data->gNorm = 0;
        data->GyroOffset[0] = 0;
        data->GyroOffset[1] = 0;
        data->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++)
        {
            BMI088_Read_Accel(hspi, data);
            g_norm_temp = sqrtf(data->Accel[0] * data->Accel[0] +
                                data->Accel[1] * data->Accel[1] +
                                data->Accel[2] * data->Accel[2]);
            data->gNorm += g_norm_temp;

            BMI088_Read_Gyro(hspi, data);
            data->GyroOffset[0] += data->Gyro[0];
            data->GyroOffset[1] += data->Gyro[1];
            data->GyroOffset[2] += data->Gyro[2];

            if (i == 0)
            {
                g_norm_max = g_norm_temp;
                g_norm_min = g_norm_temp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyro_max[j] = data->Gyro[j];
                    gyro_min[j] = data->Gyro[j];
                }
            }
            else
            {
                if (g_norm_temp > g_norm_max)
                    g_norm_max = g_norm_temp;
                if (g_norm_temp < g_norm_min)
                    g_norm_min = g_norm_temp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (data->Gyro[j] > gyro_max[j])
                        gyro_max[j] = data->Gyro[j];
                    if (data->Gyro[j] < gyro_min[j])
                        gyro_min[j] = data->Gyro[j];
                }
            }

            g_norm_diff = g_norm_max - g_norm_min;
            for (uint8_t j = 0; j < 3; j++)
                gyro_diff[j] = gyro_max[j] - gyro_min[j];

            if (g_norm_diff > 0.5f ||
                gyro_diff[0] > 0.15f ||
                gyro_diff[1] > 0.15f ||
                gyro_diff[2] > 0.15f)
                break;

            HAL_Delay(1);
        }

        data->gNorm /= (float)cali_times;
        for (uint8_t i = 0; i < 3; i++)
            data->GyroOffset[i] /= (float)cali_times;

    } while (g_norm_diff > 0.5f ||
             fabsf(data->gNorm - 9.8f) > 0.5f ||
             gyro_diff[0] > 0.15f ||
             gyro_diff[1] > 0.15f ||
             gyro_diff[2] > 0.15f ||
             fabsf(data->GyroOffset[0]) > 0.01f ||
             fabsf(data->GyroOffset[1]) > 0.01f ||
             fabsf(data->GyroOffset[2]) > 0.01f);

    data->AccelScale = 9.81f / data->gNorm;
}
