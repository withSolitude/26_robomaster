//
// Created by Lenovo on 2026/1/12.
//
#include "ita_robot.h"

//
// // 爆发优先热量打表
// static const float shoot_burst_first_heat_max[10] = {200.0f, 250.0f, 300.0f, 350.0f, 400.0f,
//                                                      450.0f, 500.0f, 550.0f, 600.0f, 650.0f};
//
// // 爆发优先冷却打表
// static const float shoot_burst_first_heat_cd[10] = {10.0f, 15.0f, 20.0f, 25.0f, 30.0f,
//                                                     35.0f, 40.0f, 45.0f, 50.0f, 60.0f};
//
// // 冷却优先热量打表
// static const float shoot_cd_first_heat_max[10] = {50.0f, 85.0f, 120.0f, 155.0f, 190.0f,
//                                                   225.0f, 260.0f, 295.0f, 330.0f, 400.0f};
//
// // 冷却优先冷却打表
// static const float shoot_cd_first_heat_cd[10] = {40.0f, 45.0f, 50.0f, 55.0f, 60.0f,
//                                                  65.0f, 70.0f, 75.0f, 80.0f, 80.0f};
// 底盘实际功率与Chassis_Power_Limit_Max对照表
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
    case 50:  return 40.0f;
    // case 55:  return 40.0f;
    // case 60:  return 45.0f;
    // case 65:  return 45.0f;
    case 70:  return 80.0f;    //84.0
    case 75:  return 86.0f;
    case 80:  return 90.5f;   //
    case 85:  return 95.0f;
    case 90:  return 100.0f;
    case 95:  return 105.0f;
    case 100: return 110.0f;
    case 105: return 115.0f;
    case 110: return 118.0f;
    case 115: return 123.0f;
    case 120: return 70.0f;//127.0f;   //82
    default:

        return 80.0f;
    }
}


/**
 *@brief 控制交互端初始化
 *
 */
void Class_Robot::Init()
{

    // 裁判系统初始化
    //referee.Init(&huart6);

    // 自瞄初始化
    AutoAim.Init();

    // 上下板通信初始化
    Slave_With_Master.Init(&hfdcan1);

    // 底盘斜坡函数初始化
    Slope_Speed_X.Init(0.009f,0.012f,Slope_First_TARGET);
    Slope_Speed_Y.Init(0.009f,0.012f,Slope_First_TARGET);
    Slope_Speed_Omega.Init(0.020f,0.020f,Slope_First_TARGET);
    Slope_Track_Omega.Init(0.012f, 0.020f, Slope_First_REAL);
    Slope_Leg_Left_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);
    Slope_Leg_Right_Angle.Init(0.0010f, 0.0015f, Slope_First_REAL);

    // 底盘跟随PID，输出是底盘目标角速度
    PID_Chassis_Follow.Init(4.5681f, 0.0f, 0.00173f, 0.0f, 0.0f, 4.0f * PI, 0.001f);

    // 遥控器初始化
    VT13.Init(&huart1);

    // 底盘初始化
    Chassis.Init();

    // 云台初始化
    Gimbal.Init();

    // 发射初始化
    Shooter.Init();

}


/**
 *@brief TIM定时器中断定时检测模块是否存活
 *
 */
void Class_Robot::TIM_1000ms_Alive_PeriodElapsedCallback()
{
    //referee.TIM_1000ms_Alive_PeriodElapsedCallback();
}
/**
 *@brief TIM定时器中断定时检测模块是否存活
 *
 */
void Class_Robot::TIM_200ms_Alive_PeriodElapsedCallback()
{

}
/**
 *@brief TIM定时器定时检测模块是否存活
 *
 */
void Class_Robot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    VT13.TIM_100ms_Alive_PeriodElapsedCallback();
    AutoAim.TIM_100ms_Alive_PeriodElapsedCallback();
    Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
    Gimbal.TIM_100ms_Alive_PeriodElapedCallback();
    Shooter.TIM_100ms_Alive_PeriodElapsedCallback();
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
    _AutoAim_Control();

    AutoAim.TIM_10ms_Send_PeriodElapsedCallback();
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

    _Status_Control();

    Gimbal.TIM_1ms_Resolution_PeriodElapedCallback();

    _Chassis_Control();

    _Gimbal_Control();
    Gimbal.TIM_1ms_Control_PeriodElapedCallback();

    _Shooter_Control();
    Shooter.TIM_1ms_Calculate_PeriodElapsedCallback();
}

/**
 *@ 自瞄
 *
 */
void Class_Robot::_AutoAim_Control()
{
    uint8_t tx_mode = AutoAim_Mode_IDLE;

    if (AutoAim_Enable == true)
    {
        if (AutoAim_Auto_Fire_Enable == true && Friction_Enable == true)
        {
            tx_mode = AutoAim_Mode_CONTROL_FIRE;
        }
        else
        {
            tx_mode = AutoAim_Mode_CONTROL_NO_FIRE;
        }
    }

    AutoAim.Set_Tx_Mode(tx_mode);

    // q[0..3] = w, x, y, z
    AutoAim.Set_Quaternion(Gimbal.Board_IMU.Get_q());

    // 当前云台角度，单位 rad
    AutoAim.Set_Now_Yaw(Gimbal.Board_IMU.Get_Yaw());
    AutoAim.Set_Now_Pitch(Gimbal.Board_IMU.Get_Roll());

    AutoAim.Set_Now_Yaw_Vel(0.0f);
    AutoAim.Set_Now_Pitch_Vel(0.0f);

    AutoAim.Set_Bullet_Speed(AutoAim_Bullet_Speed);
    AutoAim.Set_Bullet_Count(AutoAim_Bullet_Count);
}


/**
 *@brief 状态控制逻辑
 *
 */
void Class_Robot::_Status_Control()
{
    const uint8_t mode_sw = VT13.Get_Mode_Switch_Raw();

    static uint8_t last_mode_sw = 0xff;

    AutoAim_Enable = false;

    // C 档 / 遥控器离线：整车待机断控
    if (VT13.Get_Status() == Image_Transform_Status_DISABLE ||
        mode_sw == Image_Transform_Mode_SW_C)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_DISABLE);
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);

        Friction_Enable = false;
        AutoAim_Enable = false;

        Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;
        Chassis_Follow_Mode_Status = true;

        PID_Chassis_Follow.Set_Integral_Error(0.0f);

        return;
    }

    // N 档：遥控器控制
    if (mode_sw == Image_Transform_Mode_SW_N)
    {
        // 遥控器右键：小陀螺开关
        if (VT13.Get_Right_Key() == Image_Transform_Key_Status_PRESSED)
        {
            Chassis_Gyroscope_Type = Robot_Gyroscope_Type_CLOCKWISE;
            Chassis_Follow_Mode_Status = false;
            AutoAim_Enable = false;
            AutoAim_Auto_Fire_Enable = false;
        }
        else if (VT13.Get_Pause_Key() == Image_Transform_Key_Status_PRESSED)
        {
            Chassis_Gyroscope_Type = Robot_Gyroscope_Type_ANTICLOCKWISE;
            Chassis_Follow_Mode_Status = false;
            AutoAim_Enable = false;
        }
        else
        {
            Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;
            Chassis_Follow_Mode_Status = true;
            AutoAim_Enable = false;
        }

        return;
    }

    // S 档：键鼠控制
    if (mode_sw == Image_Transform_Mode_SW_S)
    {
        // Ctrl + Z：整车复位
        if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_PRESSED &&
            VT13.Get_Keyboard_Key_Z() == Image_Transform_Key_Status_PRESSED)
        {
            HAL_NVIC_SystemReset();
        }

        // 不按 Ctrl 时，正常键鼠控制
        if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
        {
            // Q/E：键鼠小陀螺，按住生效，松开恢复跟随
            if (VT13.Get_Keyboard_Key_Q() == Image_Transform_Key_Status_PRESSED)
            {
                Chassis_Gyroscope_Type = Robot_Gyroscope_Type_CLOCKWISE;
                Chassis_Follow_Mode_Status = false;
            }
            else if (VT13.Get_Keyboard_Key_E() == Image_Transform_Key_Status_PRESSED)
            {
                Chassis_Gyroscope_Type = Robot_Gyroscope_Type_ANTICLOCKWISE;
                Chassis_Follow_Mode_Status = false;
            }
            else
            {
                Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;
                Chassis_Follow_Mode_Status = true;
            }

            // 鼠标右键：只在 S 档开启自瞄
            if (VT13.Get_Mouse_Right_Key() == Image_Transform_Key_Status_TRIG_FREE_PRESSED ||
                VT13.Get_Mouse_Right_Key() == Image_Transform_Key_Status_PRESSED)
            {
                AutoAim_Enable = true;
                AutoAim_Auto_Fire_Enable = true;
            }
            else
            {
                AutoAim_Enable = false;
                AutoAim_Auto_Fire_Enable = false;
            }

            if (VT13.Get_Keyboard_Key_G() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
            {
                Gimbal_Behind_Flag ^= true;
            }

        }
        else
        {
            Chassis_Gyroscope_Type = Robot_Gyroscope_Type_DISABLE;
            Chassis_Follow_Mode_Status = true;
            AutoAim_Enable = false;
        }

        return;
    }

    // 异常挡位保护
    Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_DISABLE);
    Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
    Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
    Chassis_Follow_Mode_Status = false;
    Friction_Enable = false;
    AutoAim_Enable = false;
}

/**
 *@brief 底盘控制逻辑
 *
 */
void Class_Robot::_Chassis_Control()
{
    // 速度上限与加减速规划控制
    float tmp_chassis_velocity_max;
    float tmp_chassis_omega_max;

    tmp_chassis_velocity_max = 3.0f;
    tmp_chassis_omega_max = 3.0f * PI;

    if (Referee_IS_Valuiable())
    {
        Chassis.Set_Target_Chassis_Power_Limit_Max(Power_Calculate());
    }

    // 判断遥控器状态是否正常，不在线或急停直接断控
    if (VT13.Get_Status() == Image_Transform_Status_DISABLE ||
        VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_C)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_DISABLE);

        Chassis.Set_Target_Velocity_X(0.0f);
        Chassis.Set_Target_Velocity_Y(0.0f);
        Chassis.Set_Target_Omega(0.0f);

        PID_Chassis_Follow.Set_Integral_Error(0.0f);
        Slope_Speed_X.Set_Target(0.0f);
        Slope_Speed_Y.Set_Target(0.0f);
        Slope_Speed_Omega.Set_Target(0.0f);
        PID_Chassis_Follow.Set_Integral_Error(0.0f);

        return;
    }

    // 底盘速度期望值，方向坐标系：以云台朝向为前方
    float tmp_expect_direction_velocity_x = 0.0f;
    float tmp_expect_direction_velocity_y = 0.0f;
    float tmp_expect_direction_omega = 0.0f;

    // 底盘速度经过斜坡函数后的值，仍然是云台坐标系
    float tmp_planning_direction_velocity_x = 0.0f;
    float tmp_planning_direction_velocity_y = 0.0f;
    float tmp_planning_chassis_omega = 0.0f;

    // 最终给底盘的速度，底盘坐标系
    float tmp_chassis_velocity_x = 0.0f;
    float tmp_chassis_velocity_y = 0.0f;
    float tmp_chassis_omega = 0.0f;

    // 履带电机速度期望值
    float tmp_expect_track_wheel_omega = 0.0f;

    // 腿电机角度期望值
    float tmp_expect_leg_angle = 0.0f;

    if (VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_N)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_Wheel_Track_Leg);

        // 读遥控器数据
        float vt13_left_x = VT13.Get_Left_X();
        float vt13_left_y = VT13.Get_Left_Y();
        float vt13_dial = VT13.Get_Dial() * dial_to_angle;

        // 死区
        vt13_left_x = Math_Abs(vt13_left_x) > VT13_Dead_Zone ? vt13_left_x : 0.0f;
        vt13_left_y = Math_Abs(vt13_left_y) > VT13_Dead_Zone ? vt13_left_y : 0.0f;
        vt13_dial   = Math_Abs(vt13_dial)   > VT13_Dead_Zone ? vt13_dial   : 0.0f;

        // 左右平移
        tmp_expect_direction_velocity_x = vt13_left_x * tmp_chassis_velocity_max;
        tmp_expect_direction_velocity_y = vt13_left_y * tmp_chassis_velocity_max;

        // 腿控制
        tmp_expect_leg_angle = -vt13_dial;
    }
    else if (VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_S)
    {
        Chassis.Set_Chassis_Control_Typer(Chassis_Control_Type_Wheel_Track_Leg);

        if (VT13.Get_Keyboard_Key_CTRL() == Image_Transform_Key_Status_FREE)
        {
            if (VT13.Get_Keyboard_Key_W() == Image_Transform_Key_Status_PRESSED || VT13.Get_Keyboard_Key_W() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_expect_direction_velocity_x += tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_S() == Image_Transform_Key_Status_PRESSED || VT13.Get_Keyboard_Key_S() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_expect_direction_velocity_x -= tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_A() == Image_Transform_Key_Status_PRESSED || VT13.Get_Keyboard_Key_A() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_expect_direction_velocity_y -= tmp_chassis_velocity_max;
            }
            if (VT13.Get_Keyboard_Key_D() == Image_Transform_Key_Status_PRESSED || VT13.Get_Keyboard_Key_D() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
            {
                tmp_expect_direction_velocity_y += tmp_chassis_velocity_max;
            }
        }
    }

    // 小陀螺
    if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_CLOCKWISE)
    {
        tmp_expect_direction_omega = tmp_chassis_omega_max;
        PID_Chassis_Follow.Set_Integral_Error(0.0f);
    }
    if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_ANTICLOCKWISE)
    {
        tmp_expect_direction_omega = -tmp_chassis_omega_max;
        PID_Chassis_Follow.Set_Integral_Error(0.0f);
    }

    if (Gimbal_Behind_Flag)
    {
        Gimbal_Yaw_Relative_Zero_Rad = PI;
    }
    else
    {
        Gimbal_Yaw_Relative_Zero_Rad = 0.0f;
    }

    // 云台相对底盘的真实物理角度
    float yaw_relative_raw = Math_Modulus_Normalization(
        Gimbal.Get_Now_Yaw_Relative_Angle(),
        2.0f * PI
    );

    // 底盘跟随
    float yaw_relative_for_follow = Math_Modulus_Normalization(
        yaw_relative_raw - Gimbal_Yaw_Relative_Zero_Rad,
        2.0f * PI
    );

    // 速度坐标变换用：必须用真实物理角度，不能减 zero
    float yaw_relative_for_move = Math_Modulus_Normalization(
        -yaw_relative_raw,
        2.0f * PI
    );

    // 底盘跟随死区，防止零点附近来回抖
    // const float Chassis_Follow_Dead_Zone = 0.07f;
    //
    // if (Math_Abs(yaw_relative) < Chassis_Follow_Dead_Zone)
    // {
    //     yaw_relative = 0.0f;
    //     PID_Chassis_Follow.Set_Integral_Error(0.0f);
    // }

    // 底盘跟随
    if (Chassis_Gyroscope_Type == Robot_Gyroscope_Type_DISABLE && Chassis_Follow_Mode_Status == true)
    {
        PID_Chassis_Follow.Set_Target(0.0f);
        PID_Chassis_Follow.Set_Now(0.0f - yaw_relative_for_follow);
        PID_Chassis_Follow.TIM_Calculate_PeriodElapsedCallback();

        tmp_expect_direction_omega = PID_Chassis_Follow.Get_Out();
    }
    else
    {
        PID_Chassis_Follow.Set_Integral_Error(0.0f);
    }

    // 当前底盘速度转换到云台坐标系
    float cos_yaw_inv = arm_cos_f32(-yaw_relative_for_move);
    float sin_yaw_inv = arm_sin_f32(-yaw_relative_for_move);

    float chassis_now_vx_in_gimbal =
        Chassis.Get_Now_Velocity_x() * cos_yaw_inv -
        Chassis.Get_Now_Velocity_y() * sin_yaw_inv;

    float chassis_now_vy_in_gimbal =
        Chassis.Get_Now_Velocity_x() * sin_yaw_inv +
        Chassis.Get_Now_Velocity_y() * cos_yaw_inv;


    Slope_Speed_X.Set_Now_Real(chassis_now_vx_in_gimbal);
    Slope_Speed_Y.Set_Now_Real(chassis_now_vy_in_gimbal);
    Slope_Speed_Omega.Set_Now_Real(Chassis.Get_Now_Omega());

    Slope_Speed_X.Set_Target(tmp_expect_direction_velocity_x);
    Slope_Speed_Y.Set_Target(tmp_expect_direction_velocity_y);
    Slope_Speed_Omega.Set_Target(tmp_expect_direction_omega);

    Slope_Speed_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Speed_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Speed_Omega.TIM_Calculate_PeriodElapsedCallback();

    tmp_planning_direction_velocity_x = Slope_Speed_X.Get_Out();
    tmp_planning_direction_velocity_y = Slope_Speed_Y.Get_Out();
    tmp_planning_chassis_omega        = Slope_Speed_Omega.Get_Out();

    // =========================
    // 履带斜坡
    // =========================
    float track_now =
        0.5f * (Chassis.Get_Now_Left_Track_Wheel_Omega() +
                Chassis.Get_Now_Right_Track_Wheel_Omega());

    Slope_Track_Omega.Set_Now_Real(track_now);
    Slope_Track_Omega.Set_Target(tmp_expect_track_wheel_omega);
    Slope_Track_Omega.TIM_Calculate_PeriodElapsedCallback();

    float tmp_planning_track_wheel_omega = Slope_Track_Omega.Get_Out();

    // =========================
    // 腿电机斜坡
    // =========================
    float leg_l_now = Chassis.Leg_Wheel[0].Get_Now_Angle();
    float leg_r_now = Chassis.Leg_Wheel[1].Get_Now_Angle();

    Slope_Leg_Left_Angle.Set_Now_Real(leg_l_now);
    Slope_Leg_Right_Angle.Set_Now_Real(leg_r_now);

    Slope_Leg_Left_Angle.Set_Target(tmp_expect_leg_angle);
    Slope_Leg_Right_Angle.Set_Target(tmp_expect_leg_angle);

    Slope_Leg_Left_Angle.TIM_Calculate_PeriodElapsedCallback();
    Slope_Leg_Right_Angle.TIM_Calculate_PeriodElapsedCallback();

    float tmp_planning_leg_angle =
        0.5f * (Slope_Leg_Left_Angle.Get_Out() +
                Slope_Leg_Right_Angle.Get_Out());

    float cos_yaw = arm_cos_f32(yaw_relative_for_move);
    float sin_yaw = arm_sin_f32(yaw_relative_for_move);

    tmp_chassis_velocity_x =
        tmp_planning_direction_velocity_x * cos_yaw -
        tmp_planning_direction_velocity_y * sin_yaw;

    tmp_chassis_velocity_y =
        tmp_planning_direction_velocity_x * sin_yaw +
        tmp_planning_direction_velocity_y * cos_yaw;

    tmp_chassis_omega = tmp_planning_chassis_omega;

    // 输出到底盘
    Chassis.Set_Target_Velocity_X(tmp_chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(tmp_chassis_velocity_y);
    Chassis.Set_Target_Omega(tmp_chassis_omega);

    Chassis.Set_Track_Motor_Omega(tmp_planning_track_wheel_omega);
    Chassis.Set_Leg_Motor_Angle(tmp_planning_leg_angle);
}



/**
 *@brief 云台控制逻辑
 *
 */
void Class_Robot::_Gimbal_Control()
{
    static bool autoaim_target_inited = false;
    static uint32_t last_autoaim_crc_ok_cnt = 0;
    static float autoaim_abs_yaw_target = 0.0f;
    static float autoaim_abs_pitch_target = 0.0f;

    if (VT13.Get_Status()==Image_Transform_Status_DISABLE || VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_C)
    {
        Gimbal.Set_Chassis_Yaw_Omega(0.0f);
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
        return;
    }
    else
    {
        // 竞赛模式
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_POSITION);

        // 云台角度传参值
        float tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
        float tmp_gimbal_pitch = Gimbal.Get_Target_Pitch_Angle();
        // 前馈值
        float tmp_gimbal_yaw_feedforward_omega = 0.0f;
        float tmp_gimbal_pitch_feedforward_omega = 0.0f;
        float tmp_gimbal_pitch_feedforward_current = 0.0f;

        // 遥控器摇杆值
        float vt13_right_x = VT13.Get_Right_X();
        float vt13_right_y = VT13.Get_Right_Y();
        // 死区
        vt13_right_x = Math_Abs(vt13_right_x) > VT13_Dead_Zone ? vt13_right_x : 0.0f;
        vt13_right_y = Math_Abs(vt13_right_y) > VT13_Dead_Zone ? vt13_right_y : 0.0f;

        // 遥控控制
        const float dt = 0.001f;
        // =========================
        // N 档：只允许遥控器控制云台
        // =========================
        if ( VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_N)
        {
                      const bool autoaim_control =
                AutoAim_Enable == true &&
                AutoAim.Get_Status() == AutoAim_Status_ENABLE &&
                (AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_NO_FIRE ||
                 AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_FIRE);

            // 自瞄优先于鼠标
            if (autoaim_control)
            {

                // 第一次进入自瞄，先锁定当前目标，防止突然跳变
                if (autoaim_target_inited == false)
                {
                    autoaim_target_inited = true;

                    autoaim_abs_yaw_target = Gimbal.Get_Target_Yaw_Angle();
                    autoaim_abs_pitch_target = Gimbal.Get_Target_Pitch_Angle();

                    last_autoaim_crc_ok_cnt = AutoAim.CRC_Ok_Cnt;
                }

                // 只有收到新的视觉有效帧，才更新一次目标角
                if (AutoAim.CRC_Ok_Cnt != last_autoaim_crc_ok_cnt)
                {
                    last_autoaim_crc_ok_cnt = AutoAim.CRC_Ok_Cnt;

                    const float vision_yaw_delta = AutoAim.Get_Target_Yaw();
                    const float vision_pitch_delta = AutoAim.Get_Target_Pitch();

                    // 偏差角转绝对目标角
                    autoaim_abs_yaw_target =
                        Gimbal.Get_Now_Yaw_Angle() + vision_yaw_delta;

                    autoaim_abs_pitch_target =
                        Gimbal.Get_Now_Pitch_Angle() - vision_pitch_delta;
                }

                tmp_gimbal_yaw = autoaim_abs_yaw_target;
                tmp_gimbal_pitch = autoaim_abs_pitch_target;

            }
            else
            {
                float vt13_right_x = VT13.Get_Right_X();
                float vt13_right_y = VT13.Get_Right_Y();

                vt13_right_x = Math_Abs(vt13_right_x) > VT13_Dead_Zone ? vt13_right_x : 0.0f;
                vt13_right_y = Math_Abs(vt13_right_y) > VT13_Dead_Zone ? vt13_right_y : 0.0f;

                const float yaw_rate_max = 4.5f;
                const float pitch_rate_max = 4.5f;

                // 右摇杆 Y 控 yaw
                tmp_gimbal_yaw -= vt13_right_y * yaw_rate_max * dt;

                // 右摇杆 X 控 pitch
                tmp_gimbal_pitch = vt13_right_x * 1;
            }

        }
        else if (VT13.Get_Mode_Switch_Raw()==Image_Transform_Mode_SW_S)
        {
            float friction_omega_dt = 0.1f;
            Target_Friction_Omega += friction_omega_dt * VT13.Get_Mouse_Z();

            const bool autoaim_control =
                AutoAim_Enable == true &&
                AutoAim.Get_Status() == AutoAim_Status_ENABLE &&
                (AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_NO_FIRE ||
                 AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_FIRE);

            // 自瞄优先于鼠标
            if (autoaim_control)
            {
                // 第一次进入自瞄，先锁定当前目标，防止突然跳变
                if (autoaim_target_inited == false)
                {
                    autoaim_target_inited = true;

                    autoaim_abs_yaw_target = Gimbal.Get_Target_Yaw_Angle();
                    autoaim_abs_pitch_target = Gimbal.Get_Target_Pitch_Angle();

                    last_autoaim_crc_ok_cnt = AutoAim.CRC_Ok_Cnt;
                }

                // 只有收到新的视觉有效帧，才更新一次目标角
                if (AutoAim.CRC_Ok_Cnt != last_autoaim_crc_ok_cnt)
                {
                    last_autoaim_crc_ok_cnt = AutoAim.CRC_Ok_Cnt;

                    const float vision_yaw_delta = AutoAim.Get_Target_Yaw();
                    const float vision_pitch_delta = AutoAim.Get_Target_Pitch();

                    // 偏差角转绝对目标角
                    autoaim_abs_yaw_target =
                        Gimbal.Get_Now_Yaw_Angle() + vision_yaw_delta;

                    autoaim_abs_pitch_target =
                        Gimbal.Get_Now_Pitch_Angle() - vision_pitch_delta;
                }

                tmp_gimbal_yaw = autoaim_abs_yaw_target;
                tmp_gimbal_pitch = autoaim_abs_pitch_target;

            }
            else
            {
                autoaim_target_inited = false;
                last_autoaim_crc_ok_cnt = AutoAim.CRC_Ok_Cnt;

                float mouse_x = VT13.Get_Mouse_X();
                float mouse_y = VT13.Get_Mouse_Y();

                mouse_x = Math_Abs(mouse_x) > VT13_Mouse_Dead_Zone ? mouse_x : 0.0f;
                mouse_y = Math_Abs(mouse_y) > VT13_Mouse_Dead_Zone ? mouse_y : 0.0f;

                // 鼠标左右控制 yaw
                tmp_gimbal_yaw -= mouse_x * VT13_Mouse_Yaw_Angle_Resolution;

                // 鼠标上下控制 pitch
                tmp_gimbal_pitch += mouse_y * VT13_Mouse_Pitch_Angle_Resolution;


            }
        }

        // 给云台 yaw 速度环提供底盘角速度前馈
        Gimbal.Set_Chassis_Yaw_Omega(Chassis.Get_Now_Omega());
        // Gimbal.Set_Chassis_Yaw_Omega(0.0f);
        Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
        Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);


    }
}


void Class_Robot::_Shooter_Control()
{

    // 自瞄单发间隔，单位 ms
    static uint16_t autoaim_fire_count = 0;

    // 自瞄两发之间的最小间隔
    const uint16_t autoaim_fire_interval_ms = 200;

    if(VT13.Get_Status() == Image_Transform_Status_DISABLE || VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_C)
    {
        AutoAim_Auto_Fire_Enable = false;
        Shooter.Set_Friction_Omega(0.0f);
        autoaim_fire_count = autoaim_fire_interval_ms;
        Shooter.Set_Target_Ammo_Shoot_Frequency(0.0f);
        Friction_Enable = false;
        AutoAim_Auto_Fire_Enable = false;
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
        return ;
    }

    // 下板发来数据
    float now_heat = 0.0f;
    float now_heat_cd = 0.0f;
    float now_heat_max = 0.0f;

    // 热量通过开关
    bool heat_ok = false;

    now_heat = (float)Slave_With_Master.Get_Booster_Heat_Now();
    now_heat_cd = (float)Slave_With_Master.Get_Booster_Heat_CD();
    now_heat_max = (float)Slave_With_Master.Get_Booster_Heat_Max();

    heat_ok = (now_heat_max - now_heat > 105.0f) ? true : false;

    // 开摩擦轮

    // 单发开火开关
    bool fire_cmd = false;
    // 自动开火开关
    bool autoaim_fire_cmd = false;


    // 遥控器控制
    if (VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_N)
    {
        // AutoAim_Auto_Fire_Enable = false;

        if (VT13.Get_Left_Key() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            Friction_Enable = Friction_Enable^true;
        }
        if (VT13.Get_Trigger() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            fire_cmd = true;
        }

        if (AutoAim_Enable && AutoAim_Auto_Fire_Enable)
        {
            if (AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_FIRE)
            {
                // 第一次收到开火信号，立刻打一发
                // 后续信号持续时，按间隔打一发
                if (autoaim_fire_count >= autoaim_fire_interval_ms)
                {
                    autoaim_fire_cmd = true;
                    autoaim_fire_count = 0;
                }
                else
                {
                    autoaim_fire_count++;
                }
            }
            else
            {
                // 没有开火信号时，把计数置满
                // 下次重新收到 CONTROL_FIRE 可以立刻打一发
                autoaim_fire_count = autoaim_fire_interval_ms;
            }
        }
        else
        {
            autoaim_fire_count = autoaim_fire_interval_ms;
        }

    }

    // 键鼠控制
    if (VT13.Get_Mode_Switch_Raw() == Image_Transform_Mode_SW_S)
    {
        if (VT13.Get_Keyboard_Key_R() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            Friction_Enable = Friction_Enable^true;
        }
        if (VT13.Get_Mouse_Left_Key() == Image_Transform_Key_Status_TRIG_FREE_PRESSED)
        {
            fire_cmd = true;
        }
        else if (AutoAim_Enable && AutoAim_Auto_Fire_Enable)
        {
            if (AutoAim.Get_Rx_Mode() == AutoAim_Mode_CONTROL_FIRE)
            {
                // 第一次收到开火信号，立刻打一发
                // 后续信号持续时，按间隔打一发
                if (autoaim_fire_count >= autoaim_fire_interval_ms)
                {
                    autoaim_fire_cmd = true;
                    autoaim_fire_count = 0;
                }
                else
                {
                    autoaim_fire_count++;
                }
            }
            else
            {
                // 没有开火信号时，把计数置满
                // 下次重新收到 CONTROL_FIRE 可以立刻打一发
                autoaim_fire_count = autoaim_fire_interval_ms;
            }
        }
        else
        {
            autoaim_fire_count = autoaim_fire_interval_ms;
        }

    }

    // 下发模式
    if (Friction_Enable)
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_CEASEFIRE);
        Shooter.Set_Friction_Omega(Target_Friction_Omega);
    }
    else
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_DISABLE);
        Shooter.Set_Friction_Omega(0.0f);
    }

    if (fire_cmd && Friction_Enable)
    {
        Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_SPOT);
    }
    else if (autoaim_fire_cmd && Friction_Enable)
    {
        if (heat_ok)
        {
            Shooter.Set_Shoot_Control_Type(Shoot_Control_Type_SPOT);
        }
    }


    Shooter.Set_Now_Heat(now_heat);
    Shooter.Set_Heat_CD(now_heat_cd);
    Shooter.Set_Heat_Limit_Max(now_heat_max);
}

