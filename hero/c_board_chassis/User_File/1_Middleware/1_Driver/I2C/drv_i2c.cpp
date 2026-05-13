//
// Created by Lenovo on 2026/1/16.
//
#include "drv_i2c.h"

Struct_I2C_Manage_Object Struct_I2C1_Manage_Object = {0};
Struct_I2C_Manage_Object Struct_I2C2_Manage_Object = {0};
Struct_I2C_Manage_Object Struct_I2C3_Manage_Object = {0};

void I2C_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == nullptr)    return;

    if (hi2c->Instance == I2C1)
    {
        Struct_I2C1_Manage_Object.I2C_Handler = hi2c;
    }
    if (hi2c->Instance == I2C2)
    {
        Struct_I2C2_Manage_Object.I2C_Handler = hi2c;
    }
    if (hi2c->Instance == I2C3)
    {
        Struct_I2C3_Manage_Object.I2C_Handler = hi2c;
    }
}

HAL_StatusTypeDef I2C_Mem_Read(
    Struct_I2C_Manage_Object *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    if (hi2c == nullptr || hi2c->I2C_Handler == nullptr)
        return HAL_ERROR;

    return HAL_I2C_Mem_Read(
        hi2c->I2C_Handler,
        dev_addr,
        mem_addr,
        mem_addr_size,
        data,
        size,
        timeout);
}

HAL_StatusTypeDef I2C_Mem_Write(
    Struct_I2C_Manage_Object *hi2c,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout)
{
    if (hi2c == nullptr || hi2c->I2C_Handler == nullptr)
        return HAL_ERROR;

    return HAL_I2C_Mem_Write(
        hi2c->I2C_Handler,
        dev_addr,
        mem_addr,
        mem_addr_size,
        data,
        size,
        timeout);
}



