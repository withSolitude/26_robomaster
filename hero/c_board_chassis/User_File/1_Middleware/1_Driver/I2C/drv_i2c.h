//
// Created by Lenovo on 2026/1/16.
//

#ifndef TEST_ROBOWAKER_DRV_I2C_H
#define TEST_ROBOWAKER_DRV_I2C_H

#include "stm32f4xx_hal.h"
#include <string.h>

#define I2C_TIMEOUT 10

#include "stm32f4xx_hal.h"

/**
 *@brief I2C回调函数类型
 *
 */
typedef void (*I2C_Call_Back)();

/**
 *@brief I2C
 *
 */
struct Struct_I2C_Manage_Object
{
    I2C_HandleTypeDef *I2C_Handler;
};

extern Struct_I2C_Manage_Object Struct_I2C1_Manage_Object;
extern Struct_I2C_Manage_Object Struct_I2C2_Manage_Object;
extern Struct_I2C_Manage_Object Struct_I2C3_Manage_Object;

/* 初始化（绑定 HAL 句柄） */
void I2C_Init(I2C_HandleTypeDef *hi2c);

/* 读写接口 */
HAL_StatusTypeDef I2C_Mem_Read(
    Struct_I2C_Manage_Object *obj,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout);

HAL_StatusTypeDef I2C_Mem_Write(
    Struct_I2C_Manage_Object *obj,
    uint16_t dev_addr,
    uint16_t mem_addr,
    uint16_t mem_addr_size,
    uint8_t *data,
    uint16_t size,
    uint32_t timeout);


#endif //TEST_ROBOWAKER_DRV_I2C_H
