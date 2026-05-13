//
// Created by Lenovo on 2025/12/19.
//

//
// Created by Lenovo on 2025/12/15.
//
#include "dvc_image_transform.h"
#include <cstring>

/**
 *@brief CRC16,初值0xFFFF
 *
 */
// ===================== VT13 CRC16（按官方示例代码） =====================
static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;
    if (p_msg == nullptr) return 0xffff;

    while (len--)
    {
        data = *p_msg++;
        crc16 = (uint16_t)((crc16 >> 8) ^ crc16_tab[(crc16 ^ data) & 0x00ff]);
    }
    return crc16;
}

static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    if ((p_msg == nullptr) || (len <= 2)) return false;

    uint16_t w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    // 官方示例：CRC低字节在前，高字节在后
    return (((w_expected & 0x00ff) == p_msg[len - 2]) &&
            (((w_expected >> 8) & 0x00ff) == p_msg[len - 1]));
}
// static uint16_t CRC16_CCITT_FALSE(const uint8_t *data,uint16_t len)
// {
//     uint16_t crc = 0xFFFF;
//     for (uint16_t i = 0; i < len; i++)
//     {
//         crc ^= (uint16_t)data[i] << 8;
//         for (uint16_t j = 0; j < 8; j++)
//         {
//             if (crc & 0x8000)
//                 crc = (uint16_t)(crc << 1) ^ 0x1021;
//             else
//                 crc = (uint16_t)(crc << 1);
//         }
//     }
//     return crc;
// }
//
// // 必须等于 0x29B1
//不懂，ai写的
static uint32_t Get_BitsLE(const uint8_t *buf,uint16_t offset,uint8_t len)
{
    uint16_t byte_idx = offset / 8;
    uint8_t bit_idx = offset % 8;

    // 读取最多32bit的小端拼接，足够覆盖11/16bit字段
    uint32_t w = 0;
    w |= (uint32_t)buf[byte_idx + 0] << 0;
    w |= (uint32_t)buf[byte_idx + 1] << 8;
    w |= (uint32_t)buf[byte_idx + 2] << 16;
    w |= (uint32_t)buf[byte_idx + 3] << 24;

    w >>= bit_idx;
    if (len>=32) return w;
    return w & ((1u << len) - 1u);
}

/**
 *@brief 遥控器初始化
 *
 *@param huart 指定的UART
 *
 */
void Class_Image_Transform::Init(UART_HandleTypeDef * huart)
{
    if (huart -> Instance == USART1)
    {
        UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart -> Instance == USART2)
    {
        UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart -> Instance == USART3)
    {
        UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart -> Instance == UART5)
    {
        UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart -> Instance == USART6)
    {
        UART_Manage_Object = &UART6_Manage_Object;
    }

    //清状态
    Flag = 0;
    Pre_Flag = 0;
    Image_Transform_Status = Image_Transform_Status_DISABLE;

    std::memset(&Raw,0,sizeof(Raw));
    std::memset(&Pre_Raw,0,sizeof(Pre_Raw));
    std::memset(&Data,0,sizeof(Data));

    Parse_Len = 0;
    std::memset(Parse_Buffer, 0, sizeof(Parse_Buffer));

    Header_Cnt = 0;
    CRC_Ok_Cnt = 0;
    CRC_Fail_Cnt = 0;
}

/**
 *@brief UART接收回调函数
 *
 *@param Rx_Data接收的数据
 *
 */
void Class_Image_Transform::UART_RxCpltCallback(uint8_t *Rx_Data,uint16_t Length)
{
    // (void)Rx_Data;

    if (UART_Manage_Object == nullptr ||UART_Manage_Object->UART_Handler == nullptr) return;
    // 滑动窗口，判断遥控器是否在线
    //只在“成功解析到一帧合法VT13+CRC OK"时再算alive
    // if (Data_Process(Length))
    if (Data_Process(Rx_Data, Length))
    {
        Flag += 1;
    }
    // Flag += 1;
    //
    // Data_Process(Length);
}

/**
 *@brief TIM定时器中断定期检测遥控器是否存活
 *
 */
void Class_Image_Transform::TIM_100ms_Alive_PeriodElapsedCallback()
{
    if (UART_Manage_Object== nullptr || UART_Manage_Object->UART_Handler == nullptr) return;

    // 判断该时间段内是否接收过遥控器数据
    if (Flag == Pre_Flag)
    {
        //遥控器断联
        Image_Transform_Status = Image_Transform_Status_DISABLE;

        UART_Reinit(UART_Manage_Object->UART_Handler);
    }
    else
    {
        //遥控器保持链接
        Image_Transform_Status = Image_Transform_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 *@brief 1ms 触发计算
 *
 */
void Class_Image_Transform::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    //三档开关触发
    _Judge_ModeSwitch(&Data.Mode_Switch,Raw.Mode_SW,Pre_Raw.Mode_SW);

    //按键触发
    _Judge_Key(&Data.Pause,Raw.Pause,Pre_Raw.Pause);
    _Judge_Key(&Data.Custom_Left,Raw.Switch_1,Pre_Raw.Switch_1);
    _Judge_Key(&Data.Custom_Right,Raw.Switch_2,Pre_Raw.Switch_2);
    _Judge_Key(&Data.Trigger,Raw.Trig,Pre_Raw.Trig);

    //鼠标按键触发
    _Judge_Key(&Data.Mouse_Left_Key,Raw.Mouse_Left_Key,Pre_Raw.Mouse_Left_Key);
    _Judge_Key(&Data.Mouse_Right_Key,Raw.Mouse_Right_Key,Pre_Raw.Mouse_Right_Key);
    _Judge_Key(&Data.Mouse_Middle_Key,Raw.Mouse_Middle_Key,Pre_Raw.Mouse_Middle_Key);

    //键盘触发
    for (int i = 0; i < 16; i++)
    {
        uint8_t now = (Raw.Keyboard_Key >> i) & 0x1;
        uint8_t pre = (Pre_Raw.Keyboard_Key >> i) & 0x1;
        _Judge_Key(&Data.Keyboard_Key[i], now, pre);
    }

    //保留上一帧原始数据
    Pre_Raw = Raw;
}

/**
 *@brief 数据解析(含CRC16)
 *
 *
 *  VT13 每14ms输出21字节，帧头0xA9 0x53
 *  CRC字段在最后2字节
 *
 */
// bool Class_Image_Transform::Data_Process(uint16_t Length)
// {
//     if (UART_Manage_Object == nullptr)
//         return false;
//
//     uint8_t *rx = UART_Manage_Object->Rx_Buffer;
//     if (Length < Image_Transform_FRAME_LEN)
//         return false;
//
//     //从后面往前找,尽量取最新一帧
//     for (int i = (int)Length - Image_Transform_FRAME_LEN; i >= 0; i--)
//     {
//         const uint8_t *frm = &rx[i];
//
//         if (frm[0] != Image_Transform_HEADER_1 || frm[1] != Image_Transform_HEADER_2)
//             continue;
//
//         //解包
//         //Channel
//         Raw.Channel_0 = (uint16_t)Get_BitsLE(frm,16,11);
//         Raw.Channel_1 = (uint16_t)Get_BitsLE(frm,27,11);
//         Raw.Channel_2 = (uint16_t)Get_BitsLE(frm,38,11);
//         Raw.Channel_3 = (uint16_t)Get_BitsLE(frm,49,11);
//
//         //Mode_Switch
//         Raw.Mode_SW = (uint8_t)Get_BitsLE(frm,60,2);
//         Raw.Pause = (uint8_t)Get_BitsLE(frm,62,1);
//         Raw.Switch_1 = (uint8_t)Get_BitsLE(frm,63,1);
//         Raw.Switch_2 = (uint8_t)Get_BitsLE(frm,64,1);
//
//         //Dial
//         Raw.Dial = (uint16_t)Get_BitsLE(frm,65,11);
//         //TRIG
//         Raw.Trig = (uint8_t)Get_BitsLE(frm,76,1);
//
//         //Mouse
//         Raw.Mouse_X = (int16_t)(frm[10] | frm[11] << 8);
//         Raw.Mouse_Y = (int16_t)(frm[12] | frm[13] << 8);
//         Raw.Mouse_Z = (int16_t)(frm[14] | frm[15] << 8);
//
//         //Mouse_Switch
//         Raw.Mouse_Left_Key = (uint8_t)Get_BitsLE(frm,128,2);
//         Raw.Mouse_Right_Key = (uint8_t)Get_BitsLE(frm,130,2);
//         Raw.Mouse_Middle_Key = (uint8_t)Get_BitsLE(frm,132,2);
//
//         //Key
//         Raw.Keyboard_Key = (uint16_t)Get_BitsLE(frm,136,16);
//
//         //归一化
//         Data.Right_X = ((float)Raw.Channel_1 - Rocker_Offset) / Rocker_Num;
//         Data.Right_Y = ((float)Raw.Channel_0 - Rocker_Offset) / Rocker_Num;
//         Data.Left_X = ((float)Raw.Channel_2 - Rocker_Offset) / Rocker_Num;
//         Data.Left_Y = ((float)Raw.Channel_3 - Rocker_Offset) / Rocker_Num;
//
//         //拨轮
//         Data.Dial = ((float)Raw.Dial - Rocker_Offset) / Rocker_Num;
//
//         //鼠标
//         Data.Mouse_X = (float)Raw.Mouse_X / 32768.0f;
//         Data.Mouse_Y = (float)Raw.Mouse_Y / 32768.0f;
//         Data.Mouse_Z = (float)Raw.Mouse_Z / 32768.0f;
//
//         //返回:解析成功
//         return true;
//     }
//
//     return false;
// }
bool Class_Image_Transform::Data_Process(uint8_t *Rx_Data, uint16_t Length)
{
    if (Rx_Data == nullptr || Length == 0)
        return false;

    bool parsed_ok = false;

    // ---------- 1) 拼接本次收到的数据 ----------
    if ((uint16_t)(Parse_Len + Length) > sizeof(Parse_Buffer))
    {
        // 简单稳妥：溢出就清空重来
        Parse_Len = 0;
    }

    std::memcpy(&Parse_Buffer[Parse_Len], Rx_Data, Length);
    Parse_Len += Length;

    // ---------- 2) 在缓存中循环找帧 ----------
    while (Parse_Len >= Image_Transform_FRAME_LEN)
    {
        // 2.1 找帧头 A9 53
        int header_idx = -1;
        for (uint16_t i = 0; i + 1 < Parse_Len; i++)
        {
            if (Parse_Buffer[i] == Image_Transform_HEADER_1 &&
                Parse_Buffer[i + 1] == Image_Transform_HEADER_2)
            {
                header_idx = (int)i;
                break;
            }
        }

        // 没找到帧头：保留最后1字节（可能是A9），其余丢掉
        if (header_idx < 0)
        {
            if ((Parse_Len > 0) && (Parse_Buffer[Parse_Len - 1] == Image_Transform_HEADER_1))
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

        // 2.2 丢弃帧头前垃圾字节
        if (header_idx > 0)
        {
            std::memmove(Parse_Buffer, &Parse_Buffer[header_idx], Parse_Len - (uint16_t)header_idx);
            Parse_Len -= (uint16_t)header_idx;
        }

        // 2.3 有帧头但长度不够一帧，等下次接收
        if (Parse_Len < Image_Transform_FRAME_LEN)
        {
            break;
        }

        // 2.4 取一帧候选
        uint8_t *frm = Parse_Buffer;
        Header_Cnt++;

        // ---------- 3) CRC校验（按官方示例） ----------
        if (!verify_crc16_check_sum(frm, Image_Transform_FRAME_LEN))
        {
            CRC_Fail_Cnt++;

            // CRC失败：只丢1字节，再继续搜帧头（防止错位后漏帧）
            std::memmove(Parse_Buffer, &Parse_Buffer[1], Parse_Len - 1);
            Parse_Len -= 1;
            continue;
        }

        CRC_Ok_Cnt++;

        // ---------- 4) CRC通过，开始解包 ----------
        // 通道
        Raw.Channel_0 = (uint16_t)Get_BitsLE(frm,16,11);
        Raw.Channel_1 = (uint16_t)Get_BitsLE(frm,27,11);
        Raw.Channel_2 = (uint16_t)Get_BitsLE(frm,38,11);
        Raw.Channel_3 = (uint16_t)Get_BitsLE(frm,49,11);

        // 开关/按键
        Raw.Mode_SW  = (uint8_t)Get_BitsLE(frm,60,2);
        Raw.Pause    = (uint8_t)Get_BitsLE(frm,62,1);
        Raw.Switch_1 = (uint8_t)Get_BitsLE(frm,63,1);
        Raw.Switch_2 = (uint8_t)Get_BitsLE(frm,64,1);

        // 拨轮与扳机
        Raw.Dial = (uint16_t)Get_BitsLE(frm,65,11);
        Raw.Trig = (uint8_t)Get_BitsLE(frm,76,1);

        // 鼠标（字节对齐字段）
        Raw.Mouse_X = (int16_t)(frm[10] | (frm[11] << 8));
        Raw.Mouse_Y = (int16_t)(frm[12] | (frm[13] << 8));
        Raw.Mouse_Z = (int16_t)(frm[14] | (frm[15] << 8));

        // 鼠标按键
        Raw.Mouse_Left_Key   = (uint8_t)Get_BitsLE(frm,128,2);
        Raw.Mouse_Right_Key  = (uint8_t)Get_BitsLE(frm,130,2);
        Raw.Mouse_Middle_Key = (uint8_t)Get_BitsLE(frm,132,2);

        // 键盘
        Raw.Keyboard_Key = (uint16_t)Get_BitsLE(frm,136,16);

        // ---------- 5) 归一化（保留你当前映射顺序） ----------
        Data.Right_X = ((float)Raw.Channel_1 - Rocker_Offset) / Rocker_Num;
        Data.Right_Y = ((float)Raw.Channel_0 - Rocker_Offset) / Rocker_Num;
        Data.Left_X  = ((float)Raw.Channel_2 - Rocker_Offset) / Rocker_Num;
        Data.Left_Y  = ((float)Raw.Channel_3 - Rocker_Offset) / Rocker_Num;

        Data.Dial = ((float)Raw.Dial - Rocker_Offset) / Rocker_Num;

        Data.Mouse_X = (float)Raw.Mouse_X / 32768.0f;
        Data.Mouse_Y = (float)Raw.Mouse_Y / 32768.0f;
        Data.Mouse_Z = (float)Raw.Mouse_Z / 32768.0f;

        parsed_ok = true;

        // ---------- 6) 移除这帧，继续处理缓存中后续数据 ----------
        std::memmove(Parse_Buffer,
                     &Parse_Buffer[Image_Transform_FRAME_LEN],
                     Parse_Len - Image_Transform_FRAME_LEN);
        Parse_Len -= Image_Transform_FRAME_LEN;
    }

    return parsed_ok;
}
/**
 *@brief 判断三档开关触发判定
 *@note C/N/S分别对应0/102
 *
 */
// void Class_Image_Transform::_Judge_ModeSwitch(Enum_Image_Transform_Switch_Status *Switch,uint8_t Status,uint8_t Pre_Status)
// {
//     // Pre:C
//     if (Pre_Status == Image_Transform_Switch_Status_C)
//     {
//         if (Status == Image_Transform_Switch_Status_C)
//             *Switch = Image_Transform_Switch_Status_C;
//         else if (Status == Image_Transform_Switch_Status_N)
//             *Switch = Image_Transform_Switch_Status_TRIG_C_N;
//         else
//             *Switch = Image_Transform_Switch_Status_TRIG_N_S;
//         return;
//     }
//     // Pre:N
//     if (Pre_Status == Image_Transform_Switch_Status_N)
//     {
//         if (Status == Image_Transform_Switch_Status_C)
//             *Switch = Image_Transform_Switch_Status_TRIG_N_C;
//         else if (Status == Image_Transform_Switch_Status_N)
//             *Switch = Image_Transform_Switch_Status_N;
//         else
//             *Switch = Image_Transform_Switch_Status_TRIG_N_S;
//         return;
//     }
//     // Pre:S
//     if (Pre_Status == Image_Transform_Switch_Status_S)
//     {
//         if (Status == Image_Transform_Switch_Status_C)
//             *Switch = Image_Transform_Switch_Status_TRIG_N_C;
//         else if (Status == Image_Transform_Switch_Status_N)
//             *Switch = Image_Transform_Switch_Status_TRIG_S_N;
//         else
//             *Switch = Image_Transform_Switch_Status_S;
//         return;
//     }
//
//     //异常值保护:当作N
//     *Switch = Image_Transform_Switch_Status_C;
// }
void Class_Image_Transform::_Judge_ModeSwitch(Enum_Image_Transform_Switch_Status *Sw, uint8_t now, uint8_t pre)
{
    // now/pre: 0=C,1=N,2=S
    if (pre == 0) {
        if (now == 0) *Sw = Image_Transform_Switch_Status_C;
        else if (now == 1) *Sw = Image_Transform_Switch_Status_TRIG_C_N;
        else *Sw = Image_Transform_Switch_Status_TRIG_N_S; // 或 TRIG_C_S 你自己定义
    } else if (pre == 1) {
        if (now == 0) *Sw = Image_Transform_Switch_Status_TRIG_N_C;
        else if (now == 1) *Sw = Image_Transform_Switch_Status_N;
        else *Sw = Image_Transform_Switch_Status_TRIG_N_S;
    } else { // pre==2
        if (now == 0) *Sw = Image_Transform_Switch_Status_TRIG_N_C; // 同上：你要不要专门 TRIG_S_C
        else if (now == 1) *Sw = Image_Transform_Switch_Status_TRIG_S_N;
        else *Sw = Image_Transform_Switch_Status_S;
    }
}


/**
 *@brief 判断按键状态
 *
 */
// void Class_Image_Transform::_Judge_Key(Enum_Image_Transform_Key_Status *Key,uint8_t Status,uint8_t Pre_Status)
// {
//     if (Pre_Status == Image_Transform_BTN_RELEASED)
//     {
//         if (Status == Image_Transform_BTN_RELEASED)
//             *Key = Image_Transform_Key_Status_FREE;
//         else
//             *Key = Image_Transform_Key_Status_PRESSED;
//         return;
//     }
//
//     // Pre pressed
//     if (Status == Image_Transform_BTN_RELEASED)
//         *Key = Image_Transform_Key_Status_FREE;
//     else
//         *Key = Image_Transform_Key_Status_PRESSED;
// }
void Class_Image_Transform::_Judge_Key(Enum_Image_Transform_Key_Status *Key, uint8_t now, uint8_t pre)
{
    if (pre == 0)
    {
        if (now == 0) *Key = Image_Transform_Key_Status_FREE;
        else          *Key = Image_Transform_Key_Status_TRIG_FREE_PRESSED;
    }
    else
    {
        if (now == 0) *Key = Image_Transform_Key_Status_TRIG_PRESSED_FREE;
        else          *Key = Image_Transform_Key_Status_PRESSED;
    }
}

