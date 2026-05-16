#include "2_Device/Aimbot/dvc_aimbot.h"

#include <string.h>
#include <math.h>

static_assert(sizeof(Struct_AutoAim_USB_Tx_Data) == AUTOAIM_TX_FRAME_LEN, "AutoAim tx frame size error");
static_assert(sizeof(Struct_AutoAim_USB_Rx_Data) == AUTOAIM_RX_FRAME_LEN, "AutoAim rx frame size error");

void Class_AutoAim::Init()
{
    memset(&Tx_Data, 0, sizeof(Tx_Data));
    memset(&Rx_Data, 0, sizeof(Rx_Data));
    memset(Parse_Buffer, 0, sizeof(Parse_Buffer));

    Parse_Len = 0;

    Flag = 0;
    Pre_Flag = 0;
    Offline_Count = 0;

    Header_Cnt = 0;
    CRC_Ok_Cnt = 0;
    CRC_Fail_Cnt = 0;
    Send_Ok_Cnt = 0;
    Send_Fail_Cnt = 0;

    AutoAim_Status = AutoAim_Status_DISABLE;

    Tx_Data.head[0] = AUTOAIM_FRAME_HEAD_0;
    Tx_Data.head[1] = AUTOAIM_FRAME_HEAD_1;
    Tx_Data.mode = AutoAim_Mode_IDLE;

    Tx_Data.q[0] = 1.0f;
    Tx_Data.q[1] = 0.0f;
    Tx_Data.q[2] = 0.0f;
    Tx_Data.q[3] = 0.0f;

    Tx_Data.yaw = 0.0f;
    Tx_Data.yaw_vel = 0.0f;
    Tx_Data.pitch = 0.0f;
    Tx_Data.pitch_vel = 0.0f;
    Tx_Data.bullet_speed = 15.0f;
    Tx_Data.bullet_count = 0;

    Rx_Data.head[0] = AUTOAIM_FRAME_HEAD_0;
    Rx_Data.head[1] = AUTOAIM_FRAME_HEAD_1;
    Rx_Data.mode = AutoAim_Mode_IDLE;

    Target_Yaw = 0.0f;
    Target_Pitch = 0.0f;
}

void Class_AutoAim::USB_RxCpltCallback(uint8_t *Rx_Data_Buffer, uint16_t Length)
{
    if (Data_Process(Rx_Data_Buffer, Length))
    {
        Flag++;
    }
}

void Class_AutoAim::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (Flag == Pre_Flag)
    {
        if (Offline_Count < 0xffff)
        {
            Offline_Count++;
        }

        // 300ms 没有成功解析出新帧，认为视觉离线
        if (Offline_Count >= 3)
        {
            AutoAim_Status = AutoAim_Status_DISABLE;
            Reset_Rx_Data_To_Idle();
        }
    }
    else
    {
        Offline_Count = 0;
        AutoAim_Status = AutoAim_Status_ENABLE;
    }

    Pre_Flag = Flag;
}

void Class_AutoAim::TIM_10ms_Send_PeriodElapsedCallback()
{
    Output();

    Append_CRC16((uint8_t *)&Tx_Data, sizeof(Tx_Data));

    if (USB_Transmit_Data((uint8_t *)&Tx_Data, sizeof(Tx_Data)) == USBD_OK)
    {
        Send_Ok_Cnt++;
    }
    else
    {
        Send_Fail_Cnt++;
    }
}

bool Class_AutoAim::Data_Process(uint8_t *Rx_Data_Buffer, uint16_t Length)
{
    if (Rx_Data_Buffer == nullptr || Length == 0)
    {
        return false;
    }

    bool parsed_ok = false;

    if ((uint16_t)(Parse_Len + Length) > sizeof(Parse_Buffer))
    {
        Parse_Len = 0;
    }

    memcpy(&Parse_Buffer[Parse_Len], Rx_Data_Buffer, Length);
    Parse_Len += Length;

    while (Parse_Len >= AUTOAIM_RX_FRAME_LEN)
    {
        int header_index = -1;

        for (uint16_t i = 0; i + 1 < Parse_Len; i++)
        {
            if (Parse_Buffer[i] == AUTOAIM_FRAME_HEAD_0 &&
                Parse_Buffer[i + 1] == AUTOAIM_FRAME_HEAD_1)
            {
                header_index = i;
                break;
            }
        }

        if (header_index < 0)
        {
            if (Parse_Len > 0 && Parse_Buffer[Parse_Len - 1] == AUTOAIM_FRAME_HEAD_0)
            {
                Parse_Buffer[0] = Parse_Buffer[Parse_Len - 1];
                Parse_Len = 1;
            }
            else
            {
                Parse_Len = 0;
            }

            break;
        }

        if (header_index > 0)
        {
            memmove(Parse_Buffer,
                    &Parse_Buffer[header_index],
                    Parse_Len - header_index);

            Parse_Len -= header_index;
        }

        if (Parse_Len < AUTOAIM_RX_FRAME_LEN)
        {
            break;
        }

        Header_Cnt++;

        if (!Verify_CRC16(Parse_Buffer, AUTOAIM_RX_FRAME_LEN))
        {
            CRC_Fail_Cnt++;

            memmove(Parse_Buffer, &Parse_Buffer[1], Parse_Len - 1);
            Parse_Len -= 1;

            continue;
        }

        Struct_AutoAim_USB_Rx_Data tmp{};
        memcpy(&tmp, Parse_Buffer, sizeof(tmp));

        if (tmp.mode > AutoAim_Mode_CONTROL_FIRE)
        {
            tmp.mode = AutoAim_Mode_IDLE;
        }

        if (!isfinite(tmp.yaw) || !isfinite(tmp.pitch))
        {
            memmove(Parse_Buffer,
                    &Parse_Buffer[AUTOAIM_RX_FRAME_LEN],
                    Parse_Len - AUTOAIM_RX_FRAME_LEN);

            Parse_Len -= AUTOAIM_RX_FRAME_LEN;

            continue;
        }

        CRC_Ok_Cnt++;

        Rx_Data = tmp;

        if (Rx_Data.mode == AutoAim_Mode_IDLE)
        {
            Target_Yaw = Tx_Data.yaw;
            Target_Pitch = Tx_Data.pitch;
        }
        else
        {
            Target_Yaw = Rx_Data.yaw;
            Target_Pitch = Rx_Data.pitch;
        }

        parsed_ok = true;

        memmove(Parse_Buffer,
                &Parse_Buffer[AUTOAIM_RX_FRAME_LEN],
                Parse_Len - AUTOAIM_RX_FRAME_LEN);

        Parse_Len -= AUTOAIM_RX_FRAME_LEN;
    }

    return parsed_ok;
}
bool Class_AutoAim::Frame_Data_Is_Valid(const Struct_AutoAim_USB_Rx_Data &data)
{
    if (data.head[0] != AUTOAIM_FRAME_HEAD_0 || data.head[1] != AUTOAIM_FRAME_HEAD_1)
    {
        return false;
    }

    if (data.mode > AutoAim_Mode_CONTROL_FIRE)
    {
        return false;
    }

    if (!isfinite(data.yaw) || !isfinite(data.pitch) ||
        !isfinite(data.yaw_vel) || !isfinite(data.pitch_vel) ||
        !isfinite(data.yaw_acc) || !isfinite(data.pitch_acc))
    {
        return false;
    }

    if (fabsf(data.yaw) > AUTOAIM_YAW_ABS_MAX ||
        fabsf(data.pitch) > AUTOAIM_PITCH_ABS_MAX)
    {
        return false;
    }

    return true;
}

void Class_AutoAim::Reset_Rx_Data_To_Idle()
{
    memset(&Rx_Data, 0, sizeof(Rx_Data));

    Rx_Data.head[0] = AUTOAIM_FRAME_HEAD_0;
    Rx_Data.head[1] = AUTOAIM_FRAME_HEAD_1;
    Rx_Data.mode = AutoAim_Mode_IDLE;

    Target_Yaw = Tx_Data.yaw;
    Target_Pitch = Tx_Data.pitch;
}

void Class_AutoAim::Output()
{
    Tx_Data.head[0] = AUTOAIM_FRAME_HEAD_0;
    Tx_Data.head[1] = AUTOAIM_FRAME_HEAD_1;
}

// CRC16，poly = 0x8408，init = 0xffff，低字节在前
uint16_t Class_AutoAim::Calc_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xffff;

    if (data == nullptr)
    {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0x8408;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool Class_AutoAim::Verify_CRC16(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len <= 2)
    {
        return false;
    }

    uint16_t crc = Calc_CRC16(data, len - 2);

    return data[len - 2] == (uint8_t)(crc & 0x00ff) &&
           data[len - 1] == (uint8_t)((crc >> 8) & 0x00ff);
}

void Class_AutoAim::Append_CRC16(uint8_t *data, uint16_t len)
{
    if (data == nullptr || len <= 2)
    {
        return;
    }

    uint16_t crc = Calc_CRC16(data, len - 2);

    data[len - 2] = (uint8_t)(crc & 0x00ff);
    data[len - 1] = (uint8_t)((crc >> 8) & 0x00ff);
}