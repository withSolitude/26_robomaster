//
// Created by Lenovo on 2026/1/13.
//
#include "Gimbal_Control.h"

static inline float limit(float x,float lo,float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void Class_Gimbal::Init()
{
    Board_IMU.Init(&hspi2);
    // YAW轴电机
    Yaw_Motor_MIT.Init(&hfdcan2,0x02,0x02,Motor_DM_Control_Method_NORMAL_MIT,12.5f,30.0f,12.5f);
    Yaw_Motor_MIT.CAN_Send_Clear_Error();
    Yaw_Motor_MIT.CAN_Send_Enter();
    //
    // //PITCH轴电机
    Pitch_Motor_MIT.Init(&hfdcan3,0x00,0x00,Motor_DM_Control_Method_NORMAL_MIT,12.5f,30.0f,12.5f);
    Pitch_Motor_MIT.CAN_Send_Clear_Error();
    Pitch_Motor_MIT.CAN_Send_Enter();

    Pitch_Motor_MIT.Set_K_P(0.0f);
    Pitch_Motor_MIT.Set_K_D(0.0f);

    Class_PID_Yaw_Angle_IMU.Init(22.0f,0.0f,0.00173f,0.0f,0.0f,9.0f,0.001f);
    Class_PID_Yaw_Omega_IMU.Init(0.69437f,15.26281f,0.0,0.0f,3.05628f,7.68662f,0.001f);

    // PITCH角度环PID初始化
    Class_PID_Pitch_Angle_IMU.Init(8.49411f,0.0f,0.0009f,0.0f,0.0f,7.60364f,0.001f);
    // PITCH角速度环PID初始化
    Class_PID_Pitch_Omega_IMU.Init(1.0422f,15.0f,0.00147f,0.0f,4.55054f, 6.77388f,0.001f);

    // Pitch速度反馈低通滤波
    // 采样频率 1000Hz，截止频率 35Hz
    Filter_Pitch_Omega.Init(
        -100.0f,
        100.0f,
        Filter_Frequency_Type_LOWPASS,
        25.0f,
        FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
        1000.0f
    );

    Filter_Yaw_Omega.Init(
    -100.0f,
    100.0f,
    Filter_Frequency_Type_LOWPASS,
    25.0f,
    FREQUENCY_FILTER_DEFAULT_SAMPLING_FREQUENCY / 2.0f,
    1000.0f
);
}

void Class_Gimbal::TIM_100ms_Alive_PeriodElapedCallback()
{
    Yaw_Motor_MIT.TIM_Alive_PeriodElapsedCallback();
    Pitch_Motor_MIT.TIM_Alive_PeriodElapsedCallback();
}

void Class_Gimbal::TIM_1ms_Resolution_PeriodElapedCallback()
{
    Board_IMU.RTOS_IMU_1ms_Sampling_Task();
    Board_IMU.RTOS_IMU_1ms_EKF_Callback();
    Self_Resolution();
}

void Class_Gimbal::TIM_1ms_Control_PeriodElapedCallback()
{
    Output();

    Yaw_Motor_MIT.TIM_Send_PeriodElapsedCallback();
    Pitch_Motor_MIT.TIM_Send_PeriodElapsedCallback();
}

float Class_Gimbal::Normalize_Delta_Rad(float delta)
{
    // 归一到 [-pi, pi]
    delta = fmodf(delta, 2.0f * PI);
    if (delta > PI)  delta -= 2.0f * PI;
    if (delta < -PI) delta += 2.0f * PI;
    return delta;
}

float Class_Gimbal::Yaw_Unwrap_To_Cont_Rad(float yaw_deg)
{
    // IMU yaw: [-180,180] -> 连续角（rad）
    if (!yaw_inited)
    {
        yaw_inited   = true;
        yaw_deg_last = yaw_deg;
        yaw_rad_cont = yaw_deg * DEG_TO_RAD;
        return yaw_rad_cont;
    }

    float delta_deg = yaw_deg - yaw_deg_last;
    // 归一到 [-180, 180]
    delta_deg = Math_Modulus_Normalization(delta_deg, 360.0f);

    yaw_rad_cont += delta_deg * DEG_TO_RAD;
    yaw_deg_last = yaw_deg;
    return yaw_rad_cont;
}

void Class_Gimbal::Update_Yaw_Relative_Encoder_Angle()
{
    float raw_angle = Yaw_Motor_Direction * Yaw_Motor_MIT.Get_Now_Angle();
    Now_Yaw_Relative_Raw_Angle = raw_angle;

    float encoder_period = 2.0f * Yaw_Motor_MIT.Get_Angle_Max();

    if (encoder_period < 1.0f)
    {
        encoder_period = 2.0f * PI;
    }

    if (Yaw_Relative_Encoder_Inited == false)
    {
        Yaw_Relative_Encoder_Inited = true;
        Yaw_Relative_Raw_Last = raw_angle;
        Now_Yaw_Relative_Continuous_Angle = raw_angle;
        Now_Yaw_Relative_Angle = Math_Modulus_Normalization(Now_Yaw_Relative_Continuous_Angle, 2.0f * PI);
        Now_Yaw_Encoder_Angle = Now_Yaw_Relative_Angle;
        return;
    }

    float delta_angle = raw_angle - Yaw_Relative_Raw_Last;

    delta_angle = Math_Modulus_Normalization(delta_angle, encoder_period);

    Now_Yaw_Relative_Continuous_Angle += delta_angle;
    Yaw_Relative_Raw_Last = raw_angle;

    // 外部只用等效相对角，不用累计圈数。
    Now_Yaw_Relative_Angle = Math_Modulus_Normalization(Now_Yaw_Relative_Continuous_Angle, 2.0f * PI);

    Now_Yaw_Encoder_Angle = Now_Yaw_Relative_Angle;
}



void Class_Gimbal::Self_Resolution()
{
    Now_Yaw_Angle = Yaw_Direction*Yaw_Unwrap_To_Cont_Rad(Board_IMU.Get_Yaw())+Yaw_Offset_Rad;

    Now_Pitch_Angle = Pitch_Direction * (Board_IMU.Get_Roll() * DEG_TO_RAD) + Pitch_Offset_Rad;

    Now_Yaw_Omega_Raw = Yaw_Motor_Direction*Yaw_Motor_MIT.Get_Now_Omega();

    Now_Pitch_Omega_Raw = Pitch_Motor_Direction*Pitch_Motor_MIT.Get_Now_Omega();

    Filter_Yaw_Omega.Set_Now(Now_Yaw_Omega_Raw);
    Filter_Yaw_Omega.TIM_Calculate_PeriodElapsedCallback();
    Now_Yaw_Omega = Filter_Yaw_Omega.Get_Out();

    Filter_Pitch_Omega.Set_Now(Now_Pitch_Omega_Raw);
    Filter_Pitch_Omega.TIM_Calculate_PeriodElapsedCallback();
    Now_Pitch_Omega = Filter_Pitch_Omega.Get_Out();

    Update_Yaw_Relative_Encoder_Angle();
}

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        //清积分
        Class_PID_Yaw_Angle_IMU.Set_Integral_Error(0.0f);
        Class_PID_Yaw_Omega_IMU.Set_Integral_Error(0.0f);
        Class_PID_Pitch_Angle_IMU.Set_Integral_Error(0.0f);
        Class_PID_Pitch_Omega_IMU.Set_Integral_Error(0.0f);

        Yaw_Motor_MIT.Set_K_P(0.0f);
        Yaw_Motor_MIT.Set_K_D(0.0f);
        Pitch_Motor_MIT.Set_K_P(0.0f);
        Pitch_Motor_MIT.Set_K_D(0.0f);

        Yaw_Motor_MIT.Set_Control_Torque(0.0f);
        Pitch_Motor_MIT.Set_Control_Torque(0.0f);

        return;
    }

    // 2) IMU 未 ready：先不控（防止追默认 Target=0）
    static bool imu_latched = false;

    // 3) IMU 第一次 ready：锁一次目标 = 当前角
    if (!imu_latched)
    {
        Target_Yaw_Angle = Now_Yaw_Angle;
        imu_latched = true;
    }

    // ==============================
    // Yaw 惯量/摩擦扫频测试
    // ==============================
    if (Yaw_Inertia_Test_Enable)
    {
        Yaw_Inertia_Sweep_Control();

        // 测 Yaw 时 Pitch 正常保持

        return;
    }


    // yaw
    // yaw “就近转位”：目标永远落在当前附近的等效角
    float yaw_delta = Normalize_Delta_Rad(Target_Yaw_Angle - Now_Yaw_Angle);
    float yaw_target_near = Now_Yaw_Angle + yaw_delta;

    Class_PID_Yaw_Angle_IMU.Set_Target(yaw_target_near);
    Class_PID_Yaw_Angle_IMU.Set_Now(Now_Yaw_Angle);
    Class_PID_Yaw_Angle_IMU.TIM_Calculate_PeriodElapsedCallback();
    float yaw_omega_tar = Class_PID_Yaw_Angle_IMU.Get_Out();

    // Class_PID_Yaw_Omega_IMU.Set_Target(yaw_omega_tar);
    //  // Class_PID_Yaw_Omega_IMU.Set_Target(0.0f);
    //  Class_PID_Yaw_Omega_IMU.Set_Now(Now_Yaw_Omega);
    //  Class_PID_Yaw_Omega_IMU.TIM_Calculate_PeriodElapsedCallback();
    //  float yaw_torque = Class_PID_Yaw_Omega_IMU.Get_Out();
    //  yaw_torque = limit(yaw_torque, -Yaw_Torque_Limit, Yaw_Torque_Limit);


    // yaw_omega_tar 是云台绝对目标角速度
    // Now_Yaw_Omega 是 yaw 电机相对底盘角速度
    // 底盘旋转时，yaw 电机需要反向转来抵消底盘角速度
    float yaw_motor_omega_tar =
        yaw_omega_tar - Chassis_Yaw_Omega_FF_K * Chassis_Yaw_Omega;

    float yaw_torque = Yaw_Omega_Control_Calculate(yaw_motor_omega_tar);



    // ==============================
    // Yaw 加速度前馈
    // ==============================

    const float dt = 0.001f;

    // 目标角速度求导，得到目标角加速度
    float yaw_alpha_raw = (yaw_omega_tar - Yaw_Target_Omega_Last) / dt;
    Yaw_Target_Omega_Last = yaw_omega_tar;

    // 角加速度限幅，防止目标突变
    yaw_alpha_raw = limit(yaw_alpha_raw,
                          -Yaw_Target_Alpha_Limit,
                          Yaw_Target_Alpha_Limit);

    // 一阶低通，避免前馈力矩尖峰
    Yaw_Target_Alpha_LPF += 0.08f * (yaw_alpha_raw - Yaw_Target_Alpha_LPF);
    Yaw_Target_Alpha = Yaw_Target_Alpha_LPF;

    float yaw_acc_ff_torque = 0.0f;

        yaw_acc_ff_torque = Yaw_Acc_FF_K * Yaw_Target_Alpha;

        yaw_acc_ff_torque = limit(yaw_acc_ff_torque,
                                  -Yaw_Acc_FF_Torque_Limit,
                                  Yaw_Acc_FF_Torque_Limit);
    // ==============================
    // 最终输出
    // ==============================

     yaw_torque += yaw_acc_ff_torque;


    yaw_torque = limit(yaw_torque, -Yaw_Torque_Limit, Yaw_Torque_Limit);


    Yaw_Motor_MIT.Set_K_P(0.0f);
    Yaw_Motor_MIT.Set_K_D(0.08f);

    Yaw_Motor_MIT.Set_Control_Torque(Yaw_Motor_Direction*yaw_torque);


    // pitch
    Target_Pitch_Angle = limit(Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);

    Class_PID_Pitch_Angle_IMU.Set_Target(Target_Pitch_Angle);
    Class_PID_Pitch_Angle_IMU.Set_Now(Now_Pitch_Angle);
    Class_PID_Pitch_Angle_IMU.TIM_Calculate_PeriodElapsedCallback();
    float pitch_omega_tar = Class_PID_Pitch_Angle_IMU.Get_Out();
    //
    Class_PID_Pitch_Omega_IMU.Set_Target(pitch_omega_tar);
    Class_PID_Pitch_Omega_IMU.Set_Now(Now_Pitch_Omega);
    Class_PID_Pitch_Omega_IMU.TIM_Calculate_PeriodElapsedCallback();
    float pitch_torque = Class_PID_Pitch_Omega_IMU.Get_Out();
    pitch_torque = limit(pitch_torque, -Pitch_Torque_Limit, Pitch_Torque_Limit);

    Pitch_Gravity_Feed_Torque = -0.9062517f * sinf(Now_Pitch_Angle)-1.4232591f * cosf(Now_Pitch_Angle)+1.9607118f;


    Pitch_Motor_MIT.Set_K_P(0.0f);
    Pitch_Motor_MIT.Set_K_D(0.0f);


    Pitch_Motor_MIT.Set_Control_Torque(Pitch_Motor_Direction * pitch_torque);
}

float Class_Gimbal::Yaw_Omega_Control_Calculate(float target_omega)
{
    const float dt = 0.001f;

    float omega_error = target_omega - Now_Yaw_Omega;

    float pre_error = Yaw_Omega_Pre_Error;

    // P
    Yaw_Omega_P_Torque = Yaw_Omega_Kp * omega_error;

    // D
    float d_error = (omega_error - pre_error) / dt;
    Yaw_Omega_D_Torque = Yaw_Omega_Kd * d_error;

    // 先不含积分的输出
    float torque_no_i = Yaw_Omega_P_Torque + Yaw_Omega_D_Torque;

    // 如果目标速度和实际速度反向，说明正在刹车/过零，不继续累积积分
    bool same_direction = true;
    if (target_omega * Now_Yaw_Omega < -0.05f)
    {
        same_direction = false;
    }

    bool error_small_enough = fabsf(omega_error) < Yaw_Omega_I_Active_Error;
    bool target_not_too_fast = fabsf(target_omega) < Yaw_Omega_I_Target_Weak;

    bool not_saturated =
        fabsf(torque_no_i + Yaw_Omega_I_Torque) <
        Yaw_Omega_Out_Max * Yaw_Omega_Saturation_Protect;

    bool allow_integral =
        error_small_enough &&
        target_not_too_fast &&
        same_direction &&
        not_saturated;

    if (allow_integral)
    {
        Yaw_Omega_I_Torque += Yaw_Omega_Ki * omega_error * dt;
    }
    else
    {
        Yaw_Omega_I_Torque *= Yaw_Omega_I_Leak;
    }

    // 速度误差过零，削弱积分，防止回甩
    if (omega_error * pre_error < 0.0f)
    {
        Yaw_Omega_I_Torque *= 0.5f;
    }

    Yaw_Omega_I_Torque = limit(Yaw_Omega_I_Torque,
                               -Yaw_Omega_I_Out_Max,
                               Yaw_Omega_I_Out_Max);

    float torque = torque_no_i + Yaw_Omega_I_Torque;

    torque = limit(torque,
                   -Yaw_Omega_Out_Max,
                   Yaw_Omega_Out_Max);

    Yaw_Omega_PID_Torque = torque;

    Yaw_Omega_Pre_Error = omega_error;

    return torque;
}




void Class_Gimbal::Set_Yaw_Inertia_Test_Enable(bool enable)
{
    // 状态没有变化，不要重复清零
    if (Yaw_Inertia_Test_Enable == enable)
    {
        return;
    }

    Yaw_Inertia_Test_Enable = enable;

    Yaw_Inertia_Test_Time = 0.0f;
    Yaw_Inertia_Test_Phase = 0.0f;
    Yaw_Inertia_Test_Torque = 0.0f;
    Yaw_Inertia_Test_Freq_Now = Yaw_Inertia_Test_Freq_Min;

    Class_PID_Yaw_Angle_IMU.Set_Integral_Error(0.0f);
    Class_PID_Yaw_Omega_IMU.Set_Integral_Error(0.0f);

    if (!enable)
    {
        Yaw_Motor_MIT.Set_K_P(0.0f);
        Yaw_Motor_MIT.Set_K_D(0.0f);
        Yaw_Motor_MIT.Set_Control_Angle(0.0f);
        Yaw_Motor_MIT.Set_Control_Omega(0.0f);
        Yaw_Motor_MIT.Set_Control_Torque(0.0f);
    }
}

void Class_Gimbal::Yaw_Inertia_Sweep_Control()
{
    const float dt = 0.001f;

    Yaw_Inertia_Test_Time += dt;

    float sweep_time = Yaw_Inertia_Test_Sweep_Time;
    if (sweep_time < 1.0f)
    {
        sweep_time = 1.0f;
    }

    float t_in_sweep = fmodf(Yaw_Inertia_Test_Time, sweep_time);
    float k = t_in_sweep / sweep_time;

    // 线性扫频：Freq_Min -> Freq_Max
    Yaw_Inertia_Test_Freq_Now =
            Yaw_Inertia_Test_Freq_Min +
            (Yaw_Inertia_Test_Freq_Max - Yaw_Inertia_Test_Freq_Min) * k;

    if (Yaw_Inertia_Test_Freq_Now < 0.05f)
    {
        Yaw_Inertia_Test_Freq_Now = 0.05f;
    }

    // 相位积分
    Yaw_Inertia_Test_Phase += 2.0f * PI * Yaw_Inertia_Test_Freq_Now * dt;

    if (Yaw_Inertia_Test_Phase > 2.0f * PI)
    {
        Yaw_Inertia_Test_Phase -= 2.0f * PI;
    }

    // 力矩渐入
    float ramp = 1.0f;
    if (Yaw_Inertia_Test_Ramp_Time > 0.001f)
    {
        ramp = Yaw_Inertia_Test_Time / Yaw_Inertia_Test_Ramp_Time;
        ramp = limit(ramp, 0.0f, 1.0f);
    }

    // 正负正弦扫频力矩
    Yaw_Inertia_Test_Torque =
            ramp *
            Yaw_Inertia_Test_Torque_Max *
            sinf(Yaw_Inertia_Test_Phase);

    Yaw_Inertia_Test_Torque = limit(Yaw_Inertia_Test_Torque,
                                    -Yaw_Torque_Limit,
                                    Yaw_Torque_Limit);

    Yaw_Motor_MIT.Set_K_P(0.0f);
    Yaw_Motor_MIT.Set_K_D(0.0f);

    Yaw_Motor_MIT.Set_Control_Angle(0.0f);
    Yaw_Motor_MIT.Set_Control_Omega(0.0f);
    Yaw_Motor_MIT.Set_Control_Torque(Yaw_Motor_Direction * Yaw_Inertia_Test_Torque);
}