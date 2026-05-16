/**
 * @file drv_usb.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的USB通信初始化与配置流程
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-13 1.1 适配达妙MC02板
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/**
 * 由于Cube中 usbd_cdc_if.c 文件为C, 而我们用的是C++, 所以需将该文件扩展名改为cpp
 * static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len) 函数为实际接收回调函数, 在该文件中
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_usb.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_USB_Manage_Object USB0_Manage_Object = {nullptr};

// USB设备句柄, 由 usbd_cdc_if.c 中定义
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void USB_Init(USB_Callback Callback_Function)
{
    USB0_Manage_Object.Callback_Function = Callback_Function;

    USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
}

/**
 * @brief 发送数据帧
 *
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length)
{
    if (Data == nullptr || Length == 0)
    {
        return USBD_FAIL;
    }

    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED || hUsbDeviceHS.pClassData == nullptr)
    {
        return USBD_FAIL;
    }

    return CDC_Transmit_HS(Data, Length);
}

/**
 * @brief 自己写的USB通信下一轮接收开启前回调函数, 非HAL库回调函数
 *
 * @param Size 接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size)
{
    // 防止上位机在 USB_Init() 之前就发数据，导致 Rx_Buffer_Active 为空
    if (USB0_Manage_Object.Rx_Buffer_Active == nullptr)
    {
        USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferHS;
    }

    // 系统未初始化完成时，只重新打开接收，不进入业务解析
    if (!init_finished || USB0_Manage_Object.Callback_Function == nullptr)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return;
    }

    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;

    if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0)
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    }

    USB0_Manage_Object.Rx_Time_Stamp = SYS_Timestamp.Get_Current_Timestamp();

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);

    // 保护：USB 双缓冲是 512 字节，超过就截断，避免后续处理异常
    if (Size > USB_BUFFER_SIZE)
    {
        Size = USB_BUFFER_SIZE;
    }

    if (USB0_Manage_Object.Callback_Function != nullptr)
    {
        USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready, Size);
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/