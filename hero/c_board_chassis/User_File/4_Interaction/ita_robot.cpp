//
// Created by Lenovo on 2026/1/12.
//
#include "ita_robot.h"

/**
 *@brief 控制交互端初始化
 *
 */
void Class_Robot::Init()
{
    // 裁判系统初始化
    referee.Init(&huart6);

    // 上下板通信初始化
    Slave_With_Master.Init(&referee,&hcan1);

}


/**
 *@brief TIM定时器中断定时检测模块是否存活
 *
 */
void Class_Robot::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    referee.TIM_1000ms_Alive_PeriodElapsedCallback();
}

/**
 *@brief TIM定时器定时检测模块是否存活
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{

}

/**
 *@brief 定时器计算函数
 *
 */
void Class_Robot::TIM_100ms_Calculate_Callback()
{

}

/**
 *@brief
 *
 */
void Class_Robot::TIM_10ms_Calculate_PeriodElapsedCallback()
{
    Slave_With_Master.TIM_10ms_Send_PeriodElapsedCallback();
}

/**
 *@brief
 *
 */
void Class_Robot::TIM_2ms_Calculate_PeriodElapsedCallback()
{

}

/**
 *@brief
 *
 */
void Class_Robot::TIM_1ms_Calculate_Callback()
{

}



