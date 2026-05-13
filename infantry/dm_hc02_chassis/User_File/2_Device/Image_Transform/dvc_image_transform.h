//
// Created by Lenovo on 2025/12/15.
//

#ifndef TEST_ROBOWAKER_DVC_IMAGE_TRANSFORM_H
#define TEST_ROBOWAKER_DVC_IMAGE_TRANSFORM_H

#include "1_Middleware/1_Driver/UART/drv_uart.h"

#define Image_Transform_FRAME_LEN (21)
#define Image_Transform_HEADER_1 (0xA9)
#define Image_Transform_HEADER_2 (0x53)

#define Image_Transform_Mode_SW_C ((uint8_t)0)
#define Image_Transform_Mode_SW_N ((uint8_t)1)
#define Image_Transform_Mode_SW_S ((uint8_t)2)

#define Image_Transform_BTN_RELEASED ((uint8_t)0)
#define Image_Transform_BTN_PRESSED ((uint8_t)1)

// 键位宏定义
#define Image_Transform_KEY_W 0
#define Image_Transform_KEY_S 1
#define Image_Transform_KEY_A 2
#define Image_Transform_KEY_D 3
#define Image_Transform_KEY_SHIFT 4
#define Image_Transform_KEY_CTRL 5
#define Image_Transform_KEY_Q 6
#define Image_Transform_KEY_E 7
#define Image_Transform_KEY_R 8
#define Image_Transform_KEY_F 9
#define Image_Transform_KEY_G 10
#define Image_Transform_KEY_Z 11
#define Image_Transform_KEY_X 12
#define Image_Transform_KEY_C 13
#define Image_Transform_KEY_V 14
#define Image_Transform_KEY_B 15

/**
 *@brief 遥控器状态
 *
 */
enum Enum_Image_Transform_Status
{
    Image_Transform_Status_ENABLE=0,
    Image_Transform_Status_DISABLE
};

/**
 *@brief 三档模式状开关
 *
 */
enum Enum_Image_Transform_Switch_Status
{
    Image_Transform_Switch_Status_C=0,
    Image_Transform_Switch_Status_TRIG_C_N,
    Image_Transform_Switch_Status_TRIG_N_C,
    Image_Transform_Switch_Status_N,
    Image_Transform_Switch_Status_TRIG_N_S,
    Image_Transform_Switch_Status_TRIG_S_N,
    Image_Transform_Switch_Status_S,
};

/**
 *@brief 按键状态
 *
 */
enum Enum_Image_Transform_Key_Status
{
    Image_Transform_Key_Status_FREE=0,
    Image_Transform_Key_Status_TRIG_FREE_PRESSED,
    Image_Transform_Key_Status_TRIG_PRESSED_FREE,
    Image_Transform_Key_Status_PRESSED,
};

/**
 *@brief 遥控器数据源
 *
 */
struct Struct_Image_Transform_UART_Data
{
    uint64_t Channel_0 : 11;
    uint64_t Channel_1 : 11;
    uint64_t Channel_2 : 11;
    uint64_t Channel_3 : 11;
    uint64_t Mode_SW : 2;           //挡位切换开关，CNS那个(c:0，n:1,s:2)
    uint64_t Pause : 1;             //暂停
    uint64_t Switch_1 : 1;          //左按键
    uint64_t Switch_2 : 1;          //右按键
    uint64_t Dial : 11;             //拨轮
    uint64_t Trig : 1;              //扳机键
    int16_t Mouse_X;
    int16_t Mouse_Y;
    int16_t Mouse_Z;
    uint64_t Mouse_Left_Key : 8;
    uint64_t Mouse_Right_Key : 8;
    uint64_t Mouse_Middle_Key : 8;
    uint64_t Keyboard_Key : 16;
} __attribute__((packed));

/**
 *@brief 归一化后的数据
 *
 */
struct Struct_Image_Transform_Data
{
    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    //三档模式开关
    Enum_Image_Transform_Switch_Status Mode_Switch;

    //按键
    Enum_Image_Transform_Key_Status Pause;
    Enum_Image_Transform_Key_Status Custom_Left;
    Enum_Image_Transform_Key_Status Custom_Right;
    Enum_Image_Transform_Key_Status Trigger;

    //拨轮
    float Dial;

    //鼠标
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;

    //鼠标按键
    Enum_Image_Transform_Key_Status Mouse_Left_Key;
    Enum_Image_Transform_Key_Status Mouse_Right_Key;
    Enum_Image_Transform_Key_Status Mouse_Middle_Key;

    //键盘
    Enum_Image_Transform_Key_Status Keyboard_Key[16];

};

class Class_Image_Transform
{
public:
    void Init(UART_HandleTypeDef *huart);

    void UART_RxCpltCallback(uint8_t *Rx_Data,uint16_t Length);

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_1ms_Calculate_PeriodElapsedCallback();

    // inline Enum_Image_Transform_Status Get_Status();
    //
    // inline float Get_X();
    //
    // inline float Get_Y();
    //
    // inline Enum_Image_Transform_Switch_Status Get_Switch_Status();
    //
    // inline Enum_Image_Transform_Key_Status Get_Key_Status();
    //
    // inline float Get_Right_X();
    //
    // inline float Get_Right_Y();


    inline Enum_Image_Transform_Status Get_Status();

    inline float Get_Right_X();

    inline float Get_Right_Y();

    inline float Get_Left_X();

    inline float Get_Left_Y();

    inline Enum_Image_Transform_Switch_Status Get_Mode_Switch();

    inline Enum_Image_Transform_Key_Status Get_Pause_Key();

    inline Enum_Image_Transform_Key_Status Get_Left_Key();

    inline Enum_Image_Transform_Key_Status Get_Right_Key();

    inline Enum_Image_Transform_Key_Status Get_Trigger();

    inline float Get_Dial();

    inline float Get_Mouse_X();

    inline float Get_Mouse_Y();

    inline float Get_Mouse_Z();

    inline Enum_Image_Transform_Key_Status Get_Mouse_Left_Key();

    inline Enum_Image_Transform_Key_Status Get_Mouse_Right_Key();

    inline Enum_Image_Transform_Key_Status Get_Mouse_Middle_Key();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_W();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_S();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_A();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_D();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_SHIFT();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_CTRL();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_Q();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_E();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_R();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_F();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_G();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_Z();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_X();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_C();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_V();

    inline Enum_Image_Transform_Key_Status Get_Keyboard_Key_B();

    inline uint8_t Get_Mode_Switch_Raw();

    inline uint8_t Get_Pause_Key_Raw();

    inline uint8_t Get_Left_Key_Raw();

    inline uint8_t Get_Right_Key_Raw();

    inline uint8_t Get_Trigger_Raw();

protected:
    // 初始化相关变量

    //绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object = nullptr;

    //常量

    //遥感/拨轮标定(min=364,mid=1024,max=1684)+-660
    float Rocker_Offset = 1024.0f;
    float Rocker_Num = 660.0f;


    //内部变量

    uint8_t Parse_Buffer[128] = {0};   // 拼帧缓存
    uint16_t Parse_Len = 0;            // 当前缓存长度

    // 调试计数
    uint32_t Header_Cnt = 0;
    uint32_t CRC_Ok_Cnt = 0;
    uint32_t CRC_Fail_Cnt = 0;

    // //前一刻遥控器状态信息
    // Struct_Image_Transform_UART_Data Pre_UART_Rx_Data;

    //当前时刻的遥控器接收flag
    uint32_t Flag = 0;
    //前一时刻的遥控器接收flag
    uint32_t Pre_Flag = 0;

    //读变量

    //遥控器状态
    Enum_Image_Transform_Status Image_Transform_Status = Image_Transform_Status_DISABLE;

    //原始数据
    Struct_Image_Transform_UART_Data Raw{};
    Struct_Image_Transform_UART_Data Pre_Raw{};

    //遥控器对外接口信息
    Struct_Image_Transform_Data Data{};

    //写变量

    //读写变量

    //内部函数
    // bool Data_Process(uint16_t Length);
    bool Data_Process(uint8_t *Rx_Data, uint16_t Length);

    void _Judge_ModeSwitch(Enum_Image_Transform_Switch_Status *Sw,uint8_t Now,uint8_t Pre);

    void _Judge_Key(Enum_Image_Transform_Key_Status *Key,uint8_t Now,uint8_t Pre);


};

/**
 *@brief 获取遥控器在线状态
 *
 * @return 遥控器在线状态
 */
inline Enum_Image_Transform_Status Class_Image_Transform::Get_Status()
{
    return (Image_Transform_Status);
}

/**
 *@brief 获取遥控器右拨杆x状态
 *
 *@return float 遥控器右侧x遥感状态
 *
 */
inline float Class_Image_Transform::Get_Right_X()
{
    return (Data.Right_X);
}

/**
 *@brief 获取遥控器右拨杆y状态
 *
 *@return float 遥控器右侧y遥感状态
 *
 */
inline float Class_Image_Transform::Get_Right_Y()
{
    return (Data.Right_Y);
}

/**
 *@brief 获取遥控器左拨杆x状态
 *
 *@return float 遥控器左侧x遥感状态
 *
 */
inline float Class_Image_Transform::Get_Left_X()
{
    return (Data.Left_X);
}

/**
 *@brief 获取遥控器左拨杆y状态
 *
 *@return float 遥控器左侧y遥感状态
 *
 */
inline float Class_Image_Transform::Get_Left_Y()
{
    return (Data.Left_Y);
}

/**
 *@brief 获取遥控器三档开关状态
 *
 *@return float 遥控器右按键y遥感状态
 *
 */
inline Enum_Image_Transform_Switch_Status Class_Image_Transform::Get_Mode_Switch()
{
    return (Data.Mode_Switch);
}

/**
 *@brief 获取遥控器暂停按键状态
 *
 *@return float 遥控器暂停按键遥感状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Pause_Key()
{
    return (Data.Pause);
}

/**
 *@brief 获取遥控器左按键状态
 *
 *@return float 遥控器左按键遥感状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Left_Key()
{
    return (Data.Custom_Left);
}

/**
 *@brief 获取遥控器右按键状态
 *
 *@return float 遥控器右按键遥感状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Right_Key()
{
    return (Data.Custom_Right);
}
/**
 *@brief 获取遥控器扳机按键状态
 *
 *@return float 遥控器扳机按键遥感状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Trigger()
{
    return (Data.Trigger);
}

/**
 *@brief 获取遥控器拨轮状态
 *
 *@return float 遥控器拨轮遥感状态
 *
 */
inline float Class_Image_Transform::Get_Dial()
{
    return (Data.Dial);
}

/**
 *@brief 获取鼠标x轴状态
 *
 *@return float 鼠标x轴状态
 *
 */
inline float Class_Image_Transform::Get_Mouse_X()
{
    return (Data.Mouse_X);
}

/**
 *@brief 获取鼠标y轴状态
 *
 *@return float 鼠标y轴状态
 *
 */
inline float Class_Image_Transform::Get_Mouse_Y()
{
    return (Data.Mouse_Y);
}

/**
 *@brief 获取鼠标z轴状态
 *
 *@return float 鼠标z轴状态
 *
 */
inline float Class_Image_Transform::Get_Mouse_Z()
{
    return (Data.Mouse_Z);
}

/**
 *@brief 获取鼠标左键状态
 *
 *@return Enum_Image_Transform_Key_Status 鼠标左键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Mouse_Left_Key()
{
    return (Data.Mouse_Left_Key);
}

/**
 *@brief 获取鼠标右键状态
 *
 *@return Enum_Image_Transform_Key_Status 鼠标右键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Mouse_Right_Key()
{
    return (Data.Mouse_Right_Key);
}

/**
 *@brief 获取鼠标中键状态
 *
 *@return Enum_Image_Transform_Key_Status 鼠标中键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Mouse_Middle_Key()
{
    return (Data.Mouse_Middle_Key);
}

/**
 *@brief 获取键盘w键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘w键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_W()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_W]);
}

/**
 *@brief 获取键盘s键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘s键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_S()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_S]);
}

/**
 *@brief 获取键盘a键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘a键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_A()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_A]);
}

/**
 *@brief 获取键盘d键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘d键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_D()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_D]);
}

/**
 *@brief 获取键盘SHIFT键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘SHIFT键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_SHIFT()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_SHIFT]);
}

/**
 *@brief 获取键盘CTRL键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘CTRL键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_CTRL()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_CTRL]);
}

/**
 *@brief 获取键盘Q键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘Q键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_Q()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_Q]);
}

/**
 *@brief 获取键盘E键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘d键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_E()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_E]);
}

/**
 *@brief 获取键盘R键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘R键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_R()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_R]);
}

/**
 *@brief 获取键盘F键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘F键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_F()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_F]);
}

/**
 *@brief 获取键盘G键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘G键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_G()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_G]);
}

/**
 *@brief 获取键盘Z键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘Z键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_Z()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_Z]);
}

/**
 *@brief 获取键盘X键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘X键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_X()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_X]);
}

/**
 *@brief 获取键盘C键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘C键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_C()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_C]);
}

/**
 *@brief 获取键盘V键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘V键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_V()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_V]);
}

/**
 *@brief 获取键盘B键状态
 *
 *@return Enum_Image_Transform_Key_Status 键盘B键状态
 *
 */
inline Enum_Image_Transform_Key_Status Class_Image_Transform::Get_Keyboard_Key_B()
{
    return (Data.Keyboard_Key[Image_Transform_KEY_B]);
}

/**
 *@brief 获取遥控器三档开关状态
 *
 *@return float 遥控器右按键y遥感状态
 *
 */
inline uint8_t Class_Image_Transform::Get_Mode_Switch_Raw()
{
    return ((uint8_t)Raw.Mode_SW);
}

/**
 *@brief 获取遥控器暂停按键状态
 *
 *@return float 遥控器暂停按键遥感状态
 *
 */
inline uint8_t Class_Image_Transform::Get_Pause_Key_Raw()
{
    return ((uint8_t)Raw.Pause);
}

/**
 *@brief 获取遥控器左按键状态
 *
 *@return float 遥控器左按键遥感状态
 *
 */
inline uint8_t Class_Image_Transform::Get_Left_Key_Raw()
{
    return ((uint8_t)Raw.Switch_1);
}

/**
 *@brief 获取遥控器右按键状态
 *
 *@return float 遥控器右按键遥感状态
 *
 */
inline uint8_t Class_Image_Transform::Get_Right_Key_Raw()
{
    return ((uint8_t)Raw.Switch_2);
}
/**
 *@brief 获取遥控器扳机按键状态
 *
 *@return float 遥控器扳机按键遥感状态
 *
 */
inline uint8_t Class_Image_Transform::Get_Trigger_Raw()
{
    return ((uint8_t)Raw.Trig);
}



#endif //TEST_ROBOWAKER_DVC_IMAGE_TRANSFORM_H
