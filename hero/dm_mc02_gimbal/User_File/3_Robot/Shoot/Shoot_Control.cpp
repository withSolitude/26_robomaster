#include "Shoot_Control.h"

void Class_Shoot::Heat_Calculate()
{
    // 计算剩余热量
    float Last_Heat = Heat_Limit_Max - Now_Heat;
    if (Last_Heat < Per_Heat)
    {
        Heat_Enable = false;
    }
    else
    {
        Heat_Enable = true;
    }
}

void Class_FSM_Shoot_Anti_Jamming::TIM_Calculate_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Count_Time++;

    if (Shoot == nullptr)
    {
        return;
    }

    const float now_torque = Basic_Math_Abs(Shoot->Motor_Driver.Get_Now_Torque());
    const float now_omega = Basic_Math_Abs(Shoot->Motor_Driver.Get_Now_Omega());

    switch (Now_Status_Serial)
    {
    case FSM_Shoot_Anti_Jamming_Status_NORMAL:
    {
        if (now_torque >= Driver_Torque_Threshold && now_omega <= Driver_Omega_Threshold)
        {
            Set_Status(FSM_Shoot_Anti_Jamming_Status_JAMMING_SUSPECT);
        }
        break;
    }
    case FSM_Shoot_Anti_Jamming_Status_JAMMING_SUSPECT:
    {
        if (now_torque < Driver_Torque_Threshold || now_omega > Driver_Omega_Threshold)
        {
            Set_Status(FSM_Shoot_Anti_Jamming_Status_NORMAL);
        }
        else if (Status[Now_Status_Serial].Count_Time >= Jamming_Suspect_Time_Threshold)
        {
            Set_Status(FSM_Shoot_Anti_Jamming_Status_JAMMING_CONFIRM);
        }
        break;
    }
    case FSM_Shoot_Anti_Jamming_Status_JAMMING_CONFIRM:
    {
            Shoot->Driver_Target_Angle = Shoot->Motor_Driver.Get_Now_Angle_Continuous()
                                        - Shoot->Driver_Direction * Driver_Back_Angle;
        Shoot->Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_ANGLE);
        Shoot->Motor_Driver.Set_Target_Angle(Shoot->Driver_Target_Angle);
        Set_Status(FSM_Shoot_Anti_Jamming_Status_PROCESSING);
        break;
    }
    case FSM_Shoot_Anti_Jamming_Status_PROCESSING:
    {
        if (Status[Now_Status_Serial].Count_Time >= Jamming_Solving_Time_Threshold)
        {
            Set_Status(FSM_Shoot_Anti_Jamming_Status_NORMAL);
        }
        break;
    }
    default:
        break;
    }
}

void Class_Shoot::Init()
{
    FSM_Anti_Jamming.Shoot = this;
    FSM_Anti_Jamming.Init(FSM_Shoot_Anti_Jamming_Status_NORMAL);

    // 拨弹4340
    Motor_Driver.Init(&hfdcan2, 0x03, 0x03, Motor_DM_Control_Method_NORMAL_MIT,12.5f,30.0f,10.0f);
    Motor_Driver.CAN_Send_Clear_Error();
    Motor_Driver.CAN_Send_Enter();
    Motor_Driver.Set_K_P(0.0f);
    Motor_Driver.Set_K_D(0.0f);
    Motor_Driver.Filter_Fourier_Omega.Init(0.0f, 0.0f, Filter_Frequency_Type_LOWPASS, 15.0f, 0.0f, 1000.0f);
    Motor_Driver.PID_Angle.Init(15.0f, 0.0f, 0.06f, 0.0f, 0.0f, 20.0f,0.001f);
    Motor_Driver.PID_Omega.Init(1.1621f, 1.01284f, 0.00186f, 0.0f, 1.69838f, 9.26316f,0.001f);

    // 三个摩擦轮3508
    Motor_Friction_1.Filter_Fourier_Omega.Init(0.0f,  0.0f, Filter_Frequency_Type_LOWPASS, 15.0f, 0.0f, 1000.0f);
    Motor_Friction_2.Filter_Fourier_Omega.Init(0.0f, 0.0f, Filter_Frequency_Type_LOWPASS, 15.0f, 0.0f, 1000.0f);
    Motor_Friction_3.Filter_Fourier_Omega.Init(0.0f, 0.0f, Filter_Frequency_Type_LOWPASS, 15.0f, 0.0f, 1000.0f);

    Motor_Friction_1.PID_Omega.Init(2.5f, 0.5679f, 0.0019f, 0.0f, 1.5101f, 6.188f,0.001f);
    Motor_Friction_2.PID_Omega.Init(2.5f, 0.5679f, 0.0019f, 0.0f, 1.5101f, 6.188f,0.001f);
    Motor_Friction_3.PID_Omega.Init(2.5f, 0.5679f, 0.0019f, 0.0f, 1.5101f, 6.188f,0.001f);

    Motor_Friction_1.Init(&hfdcan3, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA, 19.2f);
    Motor_Friction_2.Init(&hfdcan3, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA, 19.2f);
    Motor_Friction_3.Init(&hfdcan3, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA, 19.2f);

    Driver_Target_Angle = 0.0f;
    Shoot_Control_Type = Shoot_Control_Type_CEASEFIRE;
}

void Class_Shoot::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Driver.TIM_Alive_PeriodElapsedCallback();
    Motor_Friction_1.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Friction_2.TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Friction_3.TIM_100ms_Alive_PeriodElapsedCallback();
}

void Class_Shoot::TIM_1ms_Calculate_PeriodElapsedCallback()
{
    Heat_Calculate();
    FSM_Anti_Jamming.TIM_Calculate_PeriodElapsedCallback();

    if (FSM_Anti_Jamming.Get_Now_Status_Serial() == FSM_Shoot_Anti_Jamming_Status_NORMAL ||
        FSM_Anti_Jamming.Get_Now_Status_Serial() == FSM_Shoot_Anti_Jamming_Status_JAMMING_SUSPECT)
    {
        Output();
    }

    Motor_Driver.TIM_Calculate_PeriodElapsedCallback();
    Motor_Friction_1.TIM_Calculate_PeriodElapsedCallback();
    Motor_Friction_2.TIM_Calculate_PeriodElapsedCallback();
    Motor_Friction_3.TIM_Calculate_PeriodElapsedCallback();
}

void Class_Shoot::Output()
{
    switch (Shoot_Control_Type)
    {
    case Shoot_Control_Type_DISABLE:
    {
        Now_Ammo_Shoot_Frequency = 0.0f;

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_1.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_2.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_3.PID_Omega.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_DISABLE);
        Motor_Driver.Set_Target_Torque(0.0f);

        Motor_Friction_1.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_2.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_3.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_1.Set_Target_Omega(0.0f);
        Motor_Friction_2.Set_Target_Omega(0.0f);
        Motor_Friction_3.Set_Target_Omega(0.0f);
        break;
    }
    case Shoot_Control_Type_CEASEFIRE:
    {
        Now_Ammo_Shoot_Frequency = 0.0f;

        Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_ANGLE);
        Motor_Driver.Set_Target_Angle(Driver_Target_Angle);

        Motor_Friction_1.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_2.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_3.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_1.Set_Target_Omega(Friction_Direction[0] * Friction_Omega);
        Motor_Friction_2.Set_Target_Omega(Friction_Direction[1] * Friction_Omega);
        Motor_Friction_3.Set_Target_Omega(Friction_Direction[2] * Friction_Omega);

        break;
    }
    case Shoot_Control_Type_SPOT:
    {
        Driver_Target_Angle = Motor_Driver.Get_Now_Angle_Continuous()+ Driver_Direction * Driver_Angle_Per_Bullet;

        Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_ANGLE);
        Motor_Driver.Set_Target_Angle(Driver_Target_Angle);

        Motor_Friction_1.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_2.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_3.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor_Friction_1.Set_Target_Omega(Friction_Direction[0] * Friction_Omega);
        Motor_Friction_2.Set_Target_Omega(Friction_Direction[1] * Friction_Omega);
        Motor_Friction_3.Set_Target_Omega(Friction_Direction[2] * Friction_Omega);

        // 单发命令发出后立即切停火，由角度环自己走完剩余角度
        Shoot_Control_Type = Shoot_Control_Type_CEASEFIRE;
        break;
    }
    case Shoot_Control_Type_AUTO:
    {
            if (Heat_Enable)
            {
                Now_Ammo_Shoot_Frequency = Target_Ammo_Shoot_Frequency;

                Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_OMEGA);
                Motor_Driver.Set_Target_Omega(
                    Driver_Direction *
                    Now_Ammo_Shoot_Frequency *
                    Driver_Angle_Per_Bullet
                );
            }
            else
            {
                Now_Ammo_Shoot_Frequency = 0.0f;
                Driver_Target_Angle = Motor_Driver.Get_Now_Angle_Continuous();

                Motor_Driver.Set_Shoot_Driver_Control_Type(Shoot_Driver_Control_Type_ANGLE);
                Motor_Driver.Set_Target_Angle(Driver_Target_Angle);
            }

            Motor_Friction_1.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Friction_2.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Friction_3.Set_Control_Method(Motor_DJI_Control_Method_OMEGA);

            Motor_Friction_1.Set_Target_Omega(Friction_Direction[0] * Friction_Omega);
            Motor_Friction_2.Set_Target_Omega(Friction_Direction[1] * Friction_Omega);
            Motor_Friction_3.Set_Target_Omega(Friction_Direction[2] * Friction_Omega);

        break;
    }
    default:
        break;
    }
}

