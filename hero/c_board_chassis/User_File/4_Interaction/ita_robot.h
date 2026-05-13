//
// Created by Lenovo on 2026/1/12.
//

#ifndef TEST_ROBOWAKER_ITA_ROBOT_H
#define TEST_ROBOWAKER_ITA_ROBOT_H

#include "3_Robot/Chassis/Chassis_Control.h"
#include "2_Device/Image_Transform/dvc_image_transform.h"
#include "2_Algorithm/Slope/alg_slope.h"
#include "1_Middleware/2_Algorithm/Timer/alg_timer.h"
#include "2_Device/Referee/dvc_referee.h"
#include "3_Robot/Slave_Connection_With_Master/Slave_With_Master.h"


/**
 *@brief 小陀螺类型
 *
 */
enum Enum_Robot_Gyroscope_Type
{
    Robot_Gyroscope_Type_DISABLE = 0,
    Robot_Gyroscope_Type_CLOCKWISE,
    Robot_Gyroscope_Type_ANTICLOCKWISE,
};

class Class_Robot;

class Class_Robot
{
public:

    // 裁判系统
    Class_Referee referee;

    // 下板与上板通信
    Class_Slave_With_Master Slave_With_Master;




    void Init();

    void Loop();

    void TIM_1000ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Alive_PeriodElapsedCallback();

    void TIM_100ms_Calculate_Callback();

    void TIM_10ms_Calculate_PeriodElapsedCallback();

    void TIM_2ms_Calculate_PeriodElapsedCallback();

    void TIM_1ms_Calculate_Callback();

protected:

    // 初始话相关常量

    // 常量

    // 内部变量


    // 读变量

    // 写变量

    // 读写变量

    // 内部函数


};



#endif //TEST_ROBOWAKER_ITA_ROBOT_H
