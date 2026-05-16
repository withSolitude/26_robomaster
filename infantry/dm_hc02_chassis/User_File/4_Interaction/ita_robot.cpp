//
// Created by Lenovo on 2026/1/12.
//
#include "ita_robot.h"
inline uint8_t Class_Robot::Referee_IS_Valuiable()
{
    uint16_t Power = Slave_With_Master.Get_Chassis_Power_Max();
    switch (Power)
    {
    case 70:
    case 75:
    case 80:
    case 85:
    case 90:
    case 95:
    case 100:
    case 105:
    case 110:
    case 115:
    case 120:
        return 1;
    default:
        return 0;
    }
}


inline float Class_Robot::Power_Calculate()
{
    uint16_t Power = Slave_With_Master.Get_Chassis_Power_Max();

    switch (Power)
    {
    case 70:  return 33.0f;    //84.0
    case 75:  return 54.0f;
    case 80:  return 56.5f;   //
    case 85:  return 60.0f;
    case 90:  return 61.8f;
    case 95:  return 64.1f;
    case 100: return 70.1f;
    case 105: return 72.0f;
    case 110: return 76.2f;
    case 115: return 80.0f;
    case 120: return 82.0f;   //82
    default:

        return 39.0f;
    }
}
/**
 *@brief 控制交互端初始化
 *
 */
void Class_Robot::Init()
{

    // 裁判系统初始化
    referee.Init(&huart1);

    // 上下板通信初始化
    Slave_With_Master.Init(&hfdcan1);
    Slave_With_Master.Referee = &referee;
    // 底盘斜坡函数初始化
    // Slope_Speed_X.Init(0.003f,0.006f,Slope_First_TARGET);
    // Slope_Speed_Y.Init(0.003f,0.006f,Slope_First_TARGET);
    // Slope_Speed_Omega.Init(0.003f,0.006f,Slope_First_TARGET);
    Slope_Speed_X.Init(0.007f,0.01f,Slope_First_TARGET);
    Slope_Speed_Y.Init(0.007f,0.01f,Slope_First_TARGET);
    Slope_Speed_Omega.Init(0.030f,0.050f,Slope_First_TARGET);
    Slope_Track_Omega.Init(0.012f, 0.020f, Slope_First_REAL);
    Slope_Leg_Left_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);
    Slope_Leg_Right_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);
    // 遥控器初始化
    VT13.Init(&huart1);

    // 底盘初始化
    Chassis.Init();

    // 云台初始化

    // 发射初始化

    // 姿态感知器初始化

    // 最大底盘功率限制初始化

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
    // VT13.TIM_100ms_Alive_PeriodElapsedCallback();

    Slave_With_Master.TIM_100ms_Alive_PeriodElapsedCallback();

    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
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
    Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
    Chassis.TIM_2ms_Control_PeriodElapsedCallback();
}

/**
 *@brief
 *
 */
void Class_Robot::TIM_1ms_Calculate_Callback()
{



    // 遥控器处理上升沿下降沿
    VT13.TIM_1ms_Calculate_PeriodElapsedCallback();

    _Chassis_Control();
}

/**
 *@brief 状态控制逻辑
 *
 */
void Class_Robot::_Status_Control()
{
    // 判断遥控器是否正常，不在线或急停直接断控
    if (VT13.Get_Status()==Image_Transform_Status_DISABLE )
        return;

    // 整车状态控制
    if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
    {

    }
}

/**
 *@brief 底盘控制逻辑
 *
 */
void Class_Robot::_Chassis_Control()
{
    // 速度上限与加减速规划控制
    float tmp_chassis_velocity_max,tmp_chassis_omega_max;
    // if (Supercap_Accelerate_Status == true)
    // {
    //
    // }
    // else
    // {
    //
    // }
    tmp_chassis_velocity_max = 2.0f;//3.0f;
    tmp_chassis_omega_max = 3.0f*PI;

    //Slope_Speed_X.Set_Increase_Value(5.0f / 1000.0f)

    // if (Referee_IS_Valuiable())
    // {
    //     Chassis.Set_Target_Chassis_Power_Limit_Max(Power_Calculate());
    // }


    // 判断遥控器状态是否正常，不在线或急停直接断控
    if (Slave_With_Master.Get_Communication_Status_With_Master() == Communication_Status_DISABLE || Slave_With_Master.Get_VT13_Robot_Status()==Robot_Status_DISABLE)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_DISABLE);


        Chassis.Set_Track_Power_Priority_Status(false);


        return;
    }
    else
    {

        // S档功率优先：前两个麦轮不参与功率分配，只给后两个麦轮和两个履带
        // 当前下板枚举中 Robot_Status_Computer 对应上板发来的第 2 个挡位，这里按 S 档使用
        Chassis.Set_Track_Power_Priority_Status(Slave_With_Master.Get_VT13_Robot_Status() == Robot_Status_Computer);



        // 比赛模式
        if (Slave_With_Master.Get_VT13_Robot_Status() == Robot_Status_Remote)
            Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_Wheel_Track_Leg);
        if (Slave_With_Master.Get_VT13_Robot_Status() == Robot_Status_Computer)
            Chassis.Set_Chassis_Control_Typer((Chassis_Control_Type_Wheel_Track_Leg));

        // 底盘速度期望值
        float tmp_expect_direction_velocity_x,tmp_expect_direction_velocity_y,tmp_expect_direction_omega;
        // 底盘速度经过斜坡函数的值
        float tmp_planning_chassis_velocity_x,tmp_planning_chassis_velocity_y,tmp_planning_chassis_omega,tmp_planning_leg_angle;
        // 底盘速度传参值
        float tmp_chassis_velocity_x,tmp_chassis_velocity_y,tmp_chassis_omega;

        // 履带转动标志位
        uint8_t Track_Move_Flag;
        // 履带电机速度期望值
        float tmp_expect_track_wheel_omega;

        // 腿电机角度期望值
        float tmp_expect_leg_angle;

        // 遥控器摇杆值
        float vt13_left_x = Slave_With_Master.Get_VT13_Left_X();
        float vt13_left_y = Slave_With_Master.Get_VT13_Left_Y();
        float vt13_yaw = - Slave_With_Master.Get_VT13_Right_Y();
        // float vt13_yaw = VT13.Get_Left_Y();
        // float vt13_left_y = VT13.Get_Right_Y();
        // float vt13_yaw = VT13.Yaw();

        static uint8_t tarck_on_sw = 0;
        // if (VT13.Get_Trigger_Raw()==Image_Transform_BTN_PRESSED)
        static uint8_t last_trig = 0;
        uint8_t now_trig = Slave_With_Master.Get_VT13_Trig();

        if (now_trig == 1 && last_trig == 0)
        {
            tarck_on_sw ^= 1;
        }

        last_trig = now_trig;


        float vt13_dial = Slave_With_Master.Get_VT13_Dial() * dial_to_angle;

        // 排除死区
        vt13_left_x = Math_Abs(vt13_left_x) > VT13_Dead_Zone ? vt13_left_x : 0.0f;
        vt13_left_y = Math_Abs(vt13_left_y) > VT13_Dead_Zone ? vt13_left_y : 0.0f;
        vt13_yaw = Math_Abs(vt13_yaw) >  VT13_Dead_Zone ? vt13_yaw : 0.0f;

        vt13_dial = Math_Abs(vt13_dial) > VT13_Dead_Zone ? vt13_dial  : 0.0f;

        // 摇杆控制
        tmp_expect_direction_velocity_x = vt13_left_x * tmp_chassis_velocity_max;
        tmp_expect_direction_velocity_y =  vt13_left_y * tmp_chassis_velocity_max;
        tmp_expect_direction_omega = vt13_yaw * tmp_chassis_omega_max;
        tmp_expect_track_wheel_omega = tarck_on_sw ? 5.0f : 0.0f;
        tmp_expect_leg_angle = vt13_dial;

        float track_now = 0.5f * (Chassis.Get_Now_Left_Track_Wheel_Omega() + Chassis.Get_Now_Right_Track_Wheel_Omega());

        Slope_Track_Omega.Set_Now_Real(track_now);
        Slope_Track_Omega.Set_Target(tmp_expect_track_wheel_omega);
        Slope_Track_Omega.TIM_Calculate_PeriodElapsedCallback();

        float tmp_planning_track_wheel_omega = Slope_Track_Omega.Get_Out();

        // 键鼠
        if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
        {
            // 没有按下Ctrl
            if (VT13.Get_Keyboard_Key_W() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_x += tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_S() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_x -= tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_A() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_y += tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_D() == Image_Transform_Key_Status_PRESSED)
            {
                tmp_expect_direction_velocity_y -= tmp_chassis_velocity_max;
            }
        }

        // 小陀螺状态控制
        if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_CLOCKWISE)
        {
            tmp_expect_direction_omega += tmp_chassis_omega_max;
        }
        if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_ANTICLOCKWISE)
        {
            tmp_expect_direction_omega -= tmp_chassis_omega_max;
        }

        // 非小陀螺则底盘跟随



        // 速度斜坡函数
        Slope_Speed_X.Set_Target(tmp_expect_direction_velocity_x);
        Slope_Speed_Y.Set_Target(tmp_expect_direction_velocity_y);
        Slope_Speed_Omega.Set_Target(tmp_expect_direction_omega);

        Slope_Speed_X.TIM_Calculate_PeriodElapsedCallback();
        Slope_Speed_Y.TIM_Calculate_PeriodElapsedCallback();
        Slope_Speed_Omega.TIM_Calculate_PeriodElapsedCallback();

        // 腿电机斜坡函数

        float leg_l_now = Chassis.Leg_Wheel[0].Get_Now_Angle();
        float leg_r_now = Chassis.Leg_Wheel[1].Get_Now_Angle();
        Slope_Leg_Left_Angle.Set_Now_Real(leg_l_now);
        Slope_Leg_Right_Angle.Set_Now_Real(leg_r_now);

        Slope_Leg_Left_Angle.Set_Target(tmp_expect_leg_angle);
        Slope_Leg_Right_Angle.Set_Target(tmp_expect_leg_angle);

        Slope_Leg_Left_Angle.TIM_Calculate_PeriodElapsedCallback();
        Slope_Leg_Right_Angle.TIM_Calculate_PeriodElapsedCallback();

        tmp_planning_leg_angle = (Slope_Leg_Left_Angle.Get_Out()+Slope_Leg_Right_Angle.Get_Out())/2;

        // 底盘相对于云台角度



        // 底盘相对于云台运动方向



        // 规划速度


        // 保存规划后结果
        tmp_planning_chassis_velocity_x = Slope_Speed_X.Get_Out();
        tmp_planning_chassis_velocity_y = Slope_Speed_Y.Get_Out();
        tmp_planning_chassis_omega = Slope_Speed_Omega.Get_Out();
        // 小陀螺模式前馈适配平动速度

        // 云台相对底盘角速度加前馈三角函数



        // 确保第一人称直线，乘上旋转矩阵


        // 角速度斜坡函数





        Chassis.Set_Target_Velocity_X(tmp_planning_chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(tmp_planning_chassis_velocity_y);
        Chassis.Set_Target_Omega(tmp_planning_chassis_omega);
        Chassis.Set_Track_Motor_Omega(-tmp_planning_track_wheel_omega);
        Chassis.Set_Leg_Motor_Angle(tmp_planning_leg_angle);

    }
}


