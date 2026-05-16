#include "Shoot_Motor_Control.h"

void Class_Shoot_Friction_Motor_DJI_C620::TIM_Calculate_PeriodElapsedCallback()
{
    Filter_Fourier_Omega.Set_Now(Get_Now_Omega());
    Filter_Fourier_Omega.TIM_Calculate_PeriodElapsedCallback();

    PID_Calculate();

    float tmp_torque = Get_Target_Torque() + Get_Feedforward_Torque();
    Basic_Math_Constrain(&tmp_torque, -Get_Current_Max(), Get_Current_Max());
    Set_Target_Torque(tmp_torque);

    Out = Get_Target_Torque() / CURRENT_TO_TORQUE / Gearbox_Rate * CURRENT_TO_OUT;
    Basic_Math_Constrain(&Out, -OUT_MAX, OUT_MAX);

    Output();

    Set_Feedforward_Omega(0.0f);
    Set_Feedforward_Torque(0.0f);
}

void Class_Shoot_Friction_Motor_DJI_C620::PID_Calculate()
{
    switch (Get_Control_Method())
    {
    case Motor_DJI_Control_Method_TORQUE:
    {
        break;
    }
    case Motor_DJI_Control_Method_OMEGA:
    {
        PID_Omega.Set_Target(Get_Target_Omega() + Get_Feedforward_Omega());
        PID_Omega.Set_Now(Filter_Fourier_Omega.Get_Out());
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Set_Target_Torque(PID_Omega.Get_Out());
        break;
    }
    case Motor_DJI_Control_Method_ANGLE:
    {
        PID_Angle.Set_Target(Get_Target_Angle());
        PID_Angle.Set_Now(Get_Now_Angle());
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        Set_Target_Omega(PID_Angle.Get_Out());

        PID_Omega.Set_Target(Get_Target_Omega() + Get_Feedforward_Omega());
        PID_Omega.Set_Now(Filter_Fourier_Omega.Get_Out());
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Set_Target_Torque(PID_Omega.Get_Out());
        break;
    }
    default:
    {
        Set_Target_Torque(0.0f);
        break;
    }
    }

    Set_Feedforward_Omega(0.0f);
    Set_Feedforward_Torque(0.0f);
}

void Class_Shoot_Driver_Motor_DM_Normal::TIM_Calculate_PeriodElapsedCallback()
{
    Filter_Fourier_Omega.Set_Now(Get_Now_Omega());
    Filter_Fourier_Omega.TIM_Calculate_PeriodElapsedCallback();
    Update_Angle_Unwrap();
    PID_Calculate();

    // 达妙MIT纯力矩输出，控制模式在 Init() 时就固定好
    Set_K_P(0.0f);
    Set_K_D(0.0f);
    Set_Control_Angle(0.0f);
    Set_Control_Omega(0.0f);
    Set_Control_Torque(Target_Torque);

    TIM_Send_PeriodElapsedCallback();
}

void Class_Shoot_Driver_Motor_DM_Normal::Update_Angle_Unwrap()
{
    const float now_angle_wrap = Get_Now_Angle();

    const float angle_max = Get_Angle_Max();
    const float angle_period = 2.0f * angle_max;
    const float half_period = angle_max;

    if (angle_period <= 0.001f)
    {
        Now_Angle_Continuous = now_angle_wrap;
        return;
    }

    if (!Angle_Unwrap_Init_Flag)
    {
        Pre_Angle_Wrap = now_angle_wrap;
        Angle_Total_Round = 0;
        Now_Angle_Continuous = now_angle_wrap;
        Angle_Unwrap_Init_Flag = true;
        return;
    }

    const float delta_angle = now_angle_wrap - Pre_Angle_Wrap;

    // 正方向跨过 +Angle_Max -> -Angle_Max
    if (delta_angle < -half_period)
    {
        Angle_Total_Round++;
    }
    // 反方向跨过 -Angle_Max -> +Angle_Max
    else if (delta_angle > half_period)
    {
        Angle_Total_Round--;
    }

    Pre_Angle_Wrap = now_angle_wrap;

    Now_Angle_Continuous = now_angle_wrap + (float)Angle_Total_Round * angle_period;
}


void Class_Shoot_Driver_Motor_DM_Normal::PID_Calculate()
{
    float torque_cmd = 0.0f;

    switch (Shoot_Driver_Control_Type)
    {
    case Shoot_Driver_Control_Type_DISABLE:
    {
        PID_Angle.Set_Integral_Error(0.0f);
        PID_Omega.Set_Integral_Error(0.0f);
        torque_cmd = 0.0f;
        break;
    }
    case Shoot_Driver_Control_Type_TORQUE:
    {
        torque_cmd = Target_Torque + Feedforward_Torque;
        break;
    }
    case Shoot_Driver_Control_Type_OMEGA:
    {
        PID_Omega.Set_Target(Target_Omega + Feedforward_Omega);
        PID_Omega.Set_Now(Filter_Fourier_Omega.Get_Out());
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();
        torque_cmd = PID_Omega.Get_Out() + Feedforward_Torque;
        break;
    }
    case Shoot_Driver_Control_Type_ANGLE:
    {
        PID_Angle.Set_Target(Target_Angle);
        PID_Angle.Set_Now(Get_Now_Angle_Continuous());
        PID_Angle.TIM_Calculate_PeriodElapsedCallback();

        float omega_target = PID_Angle.Get_Out();

        PID_Omega.Set_Target(omega_target + Feedforward_Omega);
        PID_Omega.Set_Now(Filter_Fourier_Omega.Get_Out());
        PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        torque_cmd = PID_Omega.Get_Out() + Feedforward_Torque;
        break;
    }
    default:
    {
        torque_cmd = 0.0f;
        break;
    }
    }

    Basic_Math_Constrain(&torque_cmd, -Torque_Limit, Torque_Limit);
    Target_Torque = torque_cmd;

    Feedforward_Omega = 0.0f;
    Feedforward_Torque = 0.0f;
}
