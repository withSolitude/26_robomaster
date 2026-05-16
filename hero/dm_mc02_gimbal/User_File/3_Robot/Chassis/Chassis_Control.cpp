//
// Created by Lenovo on 2025/11/19.
//
#include "Chassis_Control.h"

//底盘初始化
void Class_Chassis::Init()
{
    //履带3508PID
    Track_Wheel[0].PID_Omega.Init(0.9f,0.3f,0.0f,3.0f,2.0f,50.0f,0.002f);
    Track_Wheel[1].PID_Omega.Init(0.9f,0.3f,0.0f,3.0f,2.0f,50.0f,0.002f);



    //底盘速度xPID,
    PID_Vx.Init(9000.0f,0.0f,0.0f,0.0f,0.0f,20000.0f,0.002f);
    //底盘速度yPID
    PID_Vy.Init(9000.0f,0.0f,0.0f,0.0f,0.0f,20000.0f,0.002f);
    //底盘角速度PID
    PID_Omega.Init(2000.0f,0.0f,0.0f,0.0f,0.0f,10000.0f,0.002f);

    / 底盘轮速度环
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(0.3f,0.00f,0.0001f,0.0f,0.0f,1.0,0.002f);//4.23661f
    }


    //轮子初始化
    Track_Wheel[0].Init(&hfdcan1,Motor_DJI_ID_0x205,Motor_DJI_Control_Method_OMEGA,GEAL_RATIO);
    Track_Wheel[1].Init(&hfdcan1,Motor_DJI_ID_0x206,Motor_DJI_Control_Method_OMEGA,GEAL_RATIO);

    Leg_Wheel[0].Init(&hfdcan2,0x00,0x00,Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    Leg_Wheel[1].Init(&hfdcan2,0x01,0x01,Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);

    Motor_Wheel[0].Init(&hfdcan1,Motor_DJI_ID_0x201,Motor_DJI_Control_Method_TORQUE,GEAL_RATIO);
    Motor_Wheel[1].Init(&hfdcan1,Motor_DJI_ID_0x202,Motor_DJI_Control_Method_TORQUE,GEAL_RATIO);
    Motor_Wheel[2].Init(&hfdcan1,Motor_DJI_ID_0x203,Motor_DJI_Control_Method_TORQUE,GEAL_RATIO);
    Motor_Wheel[3].Init(&hfdcan1,Motor_DJI_ID_0x204,Motor_DJI_Control_Method_TORQUE,GEAL_RATIO);

    //达妙电机清除错误
    Leg_Wheel[0].CAN_Send_Clear_Error();
    Leg_Wheel[1].CAN_Send_Clear_Error();
    //达妙电机使能
    Leg_Wheel[0].CAN_Send_Enter();
    Leg_Wheel[1].CAN_Send_Enter();

    // RLS
    // Power_K1_Est = Motor_Wheel[0].Get_Power_K_1();
    // Power_K2_Est = Motor_Wheel[0].Get_Power_K_2();
    // RLS.Init(0.9995f, 1000.0f, Power_K1_Est, Power_K2_Est);

}

//TIM定时器中断定期检测电机是否存活
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
    for (int i = 0; i < 2; i++)
    {
        Track_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
    for (int i = 0; i < 2; i++)
    {
        Leg_Wheel[i].TIM_Alive_PeriodElapsedCallback();
    }
}

//定时器中断解算回调函数
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{
    Self_Resolution();
}

//定时器中断控制函数
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    Kinematics_Inverse_Resolution( );
    Output_To_Dynamics();
    Dynamics_Inverse_Resolution();
    Output_To_Motor();
}

//底盘实际速度解算
void Class_Chassis::Self_Resolution()
{
    Now_Vx = 0.0f;
    Now_Vy = 0.0f;
    Now_Omega = 0.0f;

    float w[4];
    //轮子转速
    w[0] = Wheel_Motor_Direction[0]*Motor_Wheel[0].Get_Now_Omega();
    w[1] = Wheel_Motor_Direction[1]*Motor_Wheel[1].Get_Now_Omega();
    w[2] = Wheel_Motor_Direction[2]*Motor_Wheel[2].Get_Now_Omega();
    w[3] = Wheel_Motor_Direction[3]*Motor_Wheel[3].Get_Now_Omega();

    //车身实际速度
    Now_Vx = (w[0] + w[1] + w[2] + w[3]) * Wheel_Radius * 0.25f * 0.70710678118f;
    Now_Vy = (w[0] - w[1] + w[2] - w[3]) * Wheel_Radius * 0.25f * 0.70710678118f;
    Now_Omega = (-w[0] + w[1] + w[2] - w[3]) * Wheel_Radius * 0.70710678118f/ (4 * MECANUM_L);

    Now_Left_Track_Wheel_Omega = Track_Motor_Direction[0] * Track_Wheel[0].Get_Now_Omega();
    Now_Right_Track_Wheel_Omega = Track_Motor_Direction[1] * Track_Wheel[1].Get_Now_Omega();
}

//运动学逆解算
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    //轮速
    float w[4];

    w[0] = (Target_Vx + Target_Vy - Target_Omega * MECANUM_L) / (Wheel_Radius * 0.70710678118f);
    w[1] = (Target_Vx - Target_Vy + Target_Omega * MECANUM_L) / (Wheel_Radius * 0.70710678118f);
    w[2] = (Target_Vx + Target_Vy + Target_Omega * MECANUM_L) / (Wheel_Radius * 0.70710678118f);
    w[3] = (Target_Vx - Target_Vy - Target_Omega * MECANUM_L) / (Wheel_Radius * 0.70710678118f);

    Target_Wheel_Omega[0] = Wheel_Motor_Direction[0]*w[0];
    Target_Wheel_Omega[1] = Wheel_Motor_Direction[1]*w[1];
    Target_Wheel_Omega[2] = Wheel_Motor_Direction[2]*w[2];
    Target_Wheel_Omega[3] = Wheel_Motor_Direction[3]*w[3];


}

//输出到动力学状态
void Class_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
            {
                for (int i = 0; i < 4; i++)
                {
                    PID_Vx.Set_Integral_Error(0.0f);
                    PID_Vy.Set_Integral_Error(0.0f);
                    PID_Omega.Set_Integral_Error(0.0f);
                }
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
                    Target_Wheel_Omega_Current[i] = 0.0f;
                    Target_Wheel_FF_Current[i] = 0.0f;
                }

                break;
            }
        case (Chassis_Control_Type_Wheel):
            {
                PID_Vx.Set_Target(Target_Vx);
                PID_Vx.Set_Now(Now_Vx);
                PID_Vx.TIM_Calculate_PeriodElapsedCallback();

                PID_Vy.Set_Target(Target_Vy);
                PID_Vy.Set_Now(Now_Vy);
                PID_Vy.TIM_Calculate_PeriodElapsedCallback();

                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.Set_Now(Now_Omega);
                PID_Omega.TIM_Calculate_PeriodElapsedCallback();


                break;
            };
        case (Chassis_Control_Type_Wheel_Track_Leg):
            {
                PID_Vx.Set_Target(Target_Vx);
                PID_Vx.Set_Now(Now_Vx);
                PID_Vx.TIM_Calculate_PeriodElapsedCallback();

                PID_Vy.Set_Target(Target_Vy);
                PID_Vy.Set_Now(Now_Vy);
                PID_Vy.TIM_Calculate_PeriodElapsedCallback();

                PID_Omega.Set_Target(Target_Omega);
                PID_Omega.Set_Now(Now_Omega);
                PID_Omega.TIM_Calculate_PeriodElapsedCallback();
                break;
            }
    }
}

//动力学逆解算
void Class_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega;

    //x,y,z的力或力矩
    force_x = PID_Vx.Get_Out();
    force_y = PID_Vy.Get_Out();
    torque_omega = PID_Omega.Get_Out();

    //分配到轮的力
    float f[4];
    f[0] = 0.25f*(force_x + force_y - torque_omega/MECANUM_L);
    f[1] = 0.25f*(force_x - force_y + torque_omega/MECANUM_L);
    f[2] = 0.25f*(force_x + force_y + torque_omega/MECANUM_L);
    f[3] = 0.25f*(force_x - force_y - torque_omega/MECANUM_L);

    Target_Wheel_FF_Current[0] = (Wheel_Motor_Direction[0]*(f[0]*Wheel_Radius/Geal_eff/KT/GEAL_RATIO));
    Target_Wheel_FF_Current[1] = (Wheel_Motor_Direction[1]*(f[1]*Wheel_Radius/Geal_eff/KT/GEAL_RATIO));
    Target_Wheel_FF_Current[2] = Wheel_Motor_Direction[2]*(f[2]*Wheel_Radius/Geal_eff/KT/GEAL_RATIO);
    Target_Wheel_FF_Current[3] = Wheel_Motor_Direction[3]*(f[3]*Wheel_Radius/Geal_eff/KT/GEAL_RATIO);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Set_Target(Target_Wheel_Omega[i]);
        Motor_Wheel[i].PID_Omega.Set_Now(Motor_Wheel[i].Get_Now_Omega());
        Motor_Wheel[i].PID_Omega.TIM_Calculate_PeriodElapsedCallback();

        Target_Wheel_Omega_Current[i] = Motor_Wheel[i].PID_Omega.Get_Out();

        Target_Wheel_Current[i] = Target_Wheel_FF_Current[i] + Target_Wheel_Omega_Current[i];
    }


    if (Target_Wheel_Current[0] > 17.0f) Target_Wheel_Current[0] = 17.0f; if (Target_Wheel_Current[0] < -17.0f) Target_Wheel_Current[0] = -17.0f;
    if (Target_Wheel_Current[1] > 17.0f) Target_Wheel_Current[1] = 17.0f; if (Target_Wheel_Current[1] < -17.0f) Target_Wheel_Current[1] = -17.0f;
    if (Target_Wheel_Current[2] > 17.0f) Target_Wheel_Current[2] = 17.0f; if (Target_Wheel_Current[2] < -17.0f) Target_Wheel_Current[2] = -17.0f;
    if (Target_Wheel_Current[3] > 17.0f) Target_Wheel_Current[3] = 17.0f; if (Target_Wheel_Current[3] < -17.0f) Target_Wheel_Current[3] = -17.0f;

    Target_Wheel_Current[0] = Basic_Math_Abs(Target_Wheel_Current[0]) > 0.1f ? Target_Wheel_Current[0] : 0.0f;
    Target_Wheel_Current[1] = Basic_Math_Abs(Target_Wheel_Current[1]) > 0.1f ? Target_Wheel_Current[1] : 0.0f;
    Target_Wheel_Current[2] = Basic_Math_Abs(Target_Wheel_Current[2]) > 0.1f ? Target_Wheel_Current[2] : 0.0f;
    Target_Wheel_Current[3] = Basic_Math_Abs(Target_Wheel_Current[3]) > 0.1f ? Target_Wheel_Current[3] : 0.0f;

}

//输出到电机
void Class_Chassis::Output_To_Motor()
{
    if (Chassis_Control_Type != Chassis_Control_Type_DISABLE)
    {
       Power_Limit_Control();
    }

    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
            {
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
                    Motor_Wheel[i].Set_Target_Torque(0.0f);
                }
                for (int i = 0; i < 2; i++)
                {
                    Track_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
                    Track_Wheel[i].Set_Target_Omega(0.0f);
                }
                for (int i = 0; i < 2; i++)
                {
                    Leg_Wheel[i].Set_Control_Angle(0.0f);
                    Leg_Wheel[i].Set_Control_Omega(0.0f);
                }
                break;
            }
        case (Chassis_Control_Type_Wheel):
            {
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
                    Motor_Wheel[i].Set_Target_Torque(Target_Wheel_Current[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    Track_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
                    Track_Wheel[i].Set_Target_Omega(Target_Track_Wheel_Omega*Track_Motor_Direction[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    Leg_Wheel[i].Set_Control_Angle(Target_Leg_Angle*Leg_Motor_Direction[i]);
                    Leg_Wheel[i].Set_Control_Omega(Leg_Omega_Limit);
                }
                break;
            }
        case (Chassis_Control_Type_Wheel_Track_Leg):
            {
                for (int i = 0; i < 4; i++)
                {
                    Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_TORQUE);
                    Motor_Wheel[i].Set_Target_Torque(Target_Wheel_Current[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    Track_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
                    Track_Wheel[i].Set_Target_Omega(Target_Track_Wheel_Omega*Track_Motor_Direction[i]);
                }
                for (int i = 0; i < 2; i++)
                {
                    Leg_Wheel[i].Set_Control_Angle(Target_Leg_Angle*Leg_Motor_Direction[i]);
                    Leg_Wheel[i].Set_Control_Omega(Leg_Omega_Limit);
                }
                break;
            }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }
    for (int i = 0; i < 2; i++)
    {
        Track_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }
    for (int i = 0; i < 2; i++)
    {
        Leg_Wheel[i].TIM_Send_PeriodElapsedCallback();
    }

}


// 功率控制
 // 模型：P = K0*I*ω + K1*ω^2 + K2*I^2 + A
 //RLS
 void Class_Chassis::Power_Limit_Control()
 {
     // 功率上限 <=0 ，直接不限制
     if (Chassis_Power_Limit_Max <= 0.1f)
     {
         Chassis_Power_Est_Cmd = 0.0f;
         Chassis_Power_Est_Limited = 0.0f;
         return;
     }

     float cmd_current[4];
     float limited_current[4];
     float cmd_torque[4];
     float limited_torque[4];
     float cmd_power[4];
     float err_av[4];

     float sum_cmd_power_pos = 0.0f;     //仅统计正功率
     float sum_cmd_power_all = 0.0f;     //含负功率
     float regen_power = 0.0f;           //负功率带来的回收预算
     float sum_error = 0.0f;

    // 计算指令功率/误差
    for (int i = 0; i < 4; i++)
    {
        cmd_torque[i] = Target_Wheel_Current[i];   // 单位保持 A
        cmd_current[i] = cmd_torque[i] / GEAL_RATIO / (0.3f / (3591.0f / 187.0f));

        float now_omega_wheel_raw = Motor_Wheel[i].Get_Now_Omega();
        float now_omega_motor_raw = now_omega_wheel_raw * GEAL_RATIO;
        float target_omega_wheel_raw = Target_Wheel_Omega[i];

        float error = target_omega_wheel_raw - now_omega_wheel_raw;
        err_av[i] = Basic_Math_Abs(error);
        sum_error += err_av[i];

        float k0 = Motor_Wheel[i].Get_Power_K_0();
        float k1 = (Power_RLS_ENABLE != 0) ? Power_K1_Est : Motor_Wheel[i].Get_Power_K_1();
        float k2 = (Power_RLS_ENABLE != 0) ? Power_K2_Est : Motor_Wheel[i].Get_Power_K_2();
        float A  = Motor_Wheel[i].Get_Power_A();

        float p = k0 * cmd_current[i] * now_omega_motor_raw
                + k1 * now_omega_motor_raw * now_omega_motor_raw
                + k2 * cmd_current[i] * cmd_current[i]
                + A;

        cmd_power[i] = p;

        sum_cmd_power_all += p;
        if (p > 0.0f)
        {
            sum_cmd_power_pos += p;
        }
        else
        {
            regen_power += (-p);
        }
    }
     Chassis_Power_Est_Cmd = sum_cmd_power_all;

     // 没有正功率可分配，直接退出
     if (sum_cmd_power_pos < 1e-3f)
     {
         Chassis_Power_Est_Limited = sum_cmd_power_all;
         return;
     }

     // 可分配功率 = 限制上限 + 回收功率
        float reserve = 5.0f + 0.04f * Chassis_Power_Limit_Max;
        float effective_limit = Chassis_Power_Limit_Max - reserve;
        if (effective_limit < 5.0f) effective_limit = 5.0f;

        float allocatable_power = effective_limit + regen_power;
     // float allocatable_power = Chassis_Power_Limit_Max + regen_power;

     // 如果已经在预算内，不限制
     if (sum_cmd_power_pos <= allocatable_power)
     {
         for (int i= 0; i < 4; i++)
         {
             limited_current[i] = cmd_current[i];
         }
         Chassis_Power_Est_Limited = sum_cmd_power_all;
         return;
     }

     // 误差权重/功率权重平滑切换
     float avg_error = sum_error * 0.25f;
     float error_confidence = 0.0f;
     if (Power_Distribute_Error_Set > Power_Distribute_Prop_Set)
     {
         error_confidence = (avg_error - Power_Distribute_Prop_Set) / (Power_Distribute_Error_Set - Power_Distribute_Prop_Set);
         error_confidence = Basic_Math_Constrain(&error_confidence,0.0f,1.0f);
     }

     // 按权重分配功率，并反解得到新的电流指令
     for (int i = 0; i < 4; i++)
     {
         // 负功率不做限幅
         if (cmd_power[i] <= 0.0f)
         {
             limited_current[i] = cmd_current[i];
             continue;
         }

         float w_error = (sum_error > 1e-6f) ? (err_av[i] / sum_error) : 0.25f;

         float w_prop = (sum_cmd_power_pos > 1e-6f) ? (cmd_power[i] / sum_cmd_power_pos) : 0.25f;
         float w = error_confidence * w_error + (1.0f - error_confidence) * w_prop;

         float p_share = w * allocatable_power;

         float now_omega_wheel_raw = Motor_Wheel[i].Get_Now_Omega();
         float now_omega_motor_raw = now_omega_wheel_raw * GEAL_RATIO;

         float k0 = Motor_Wheel[i].Get_Power_K_0();
         float k1 = (Power_RLS_ENABLE != 0) ? Power_K1_Est : Motor_Wheel[i].Get_Power_K_1();
         float k2 = (Power_RLS_ENABLE != 0) ? Power_K2_Est : Motor_Wheel[i].Get_Power_K_2();
         float A  = Motor_Wheel[i].Get_Power_A();

         float p_idle = k1 * now_omega_motor_raw * now_omega_motor_raw + A;
         if (p_share <= p_idle + 1e-4f)
         {
             limited_current[i] = 0.0f;
             continue;
         }

         float a = k2;
         float b = k0 * now_omega_motor_raw;
         float c = k1 * now_omega_motor_raw * now_omega_motor_raw + A - p_share;
         float I_new = cmd_current[i];

         // 退化为一次
         if (Basic_Math_Abs(a) < 1e-9f)
         {
             if (Basic_Math_Abs(b) > 1e-9f)
             {
                 I_new = -c / b;
             }
         }
         else
         {
             float delta = b * b - 4.0f * a * c;
             if (delta < 0.0f)
                 delta = 0.0f;
             float sqrt_delta = sqrt(delta);

             float r1 = (-b + sqrt_delta) / (2.0f * a);
             float r2 = (-b - sqrt_delta) / (2.0f * a);

             // 选择与原指令同号且更加接近原指令的根
             float cmd = cmd_current[i];
             float pick = r1;

             //优先同号
             uint8_t r1_ok = (cmd >= 0.0f) ? (r1 >= 0.0f) : (r1 <= 0.0f);
             uint8_t r2_ok = (cmd >= 0.0f) ? (r2 >= 0.0f) : (r2 <= 0.0f);

             if (r1_ok && r2_ok)
             {
                 pick = (Basic_Math_Abs(r1 - cmd) < Basic_Math_Abs(r2 - cmd)) ? r1 : r2;
             }
             else if (r1_ok)
             {
                 pick = r1;
             }
             else if (r2_ok)
             {
                 pick = r2;
             }
             else
             {
                 // 都不同号，选幅值更小的
                 pick = (Basic_Math_Abs(r1) < Basic_Math_Abs(r2)) ? r1 : r2;
             }

             I_new = pick;
         }

         // 不允许因为分配导致比原指令更大
         if (Basic_Math_Abs(I_new) > Basic_Math_Abs(cmd_current[i]))
         {
             I_new = cmd_current[i];
         }

         // 限幅到电机允许电流
         float I_max = Motor_Wheel[i].Get_Current_Max() / (GEAL_RATIO * (0.3f / (3591.0f / 187.0f)));
         I_new = Basic_Math_Constrain(&I_new, -I_max, I_max);
         limited_current[i] = I_new;
     }

     // 写回限制后的电流
     for (int i = 0; i < 4; i++)
     {
         // cmd_torque[i] = Target_Wheel_Current[i];   // 单位保持 A
         // cmd_current[i] = cmd_torque[i] / GEAL_RATIO / (0.3f / (3591.0f / 187.0f));
         Target_Wheel_Current[i] = limited_current[i] * GEAL_RATIO * (0.3f / (3591.0f / 187.0f));
     }

     // 重新估计限制后的总功率 (调试用)
     float sum_limited = 0.0f;
     for (int i = 0; i < 4; i++)
     {
         float now_omega_wheel_raw = Motor_Wheel[i].Get_Now_Omega();
         float now_omega_motor_raw = now_omega_wheel_raw * GEAL_RATIO;
         float k0 = Motor_Wheel[i].Get_Power_K_0();
         float k1 = (Power_RLS_ENABLE != 0) ? Power_K1_Est : Motor_Wheel[i].Get_Power_K_1();
         float k2 = (Power_RLS_ENABLE != 0) ? Power_K2_Est : Motor_Wheel[i].Get_Power_K_2();
         float A = Motor_Wheel[i].Get_Power_A();
         float I = limited_current[i];
         float P = k0 * I * now_omega_motor_raw
                 + k1 * now_omega_motor_raw * now_omega_motor_raw
                 + k2 * I * I
                 + A;

         sum_limited += P;
     }
     Chassis_Power_Est_Limited = sum_limited;
 }