// #include "test.h"
// #include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
// #include "2_Device/Serialplot/dvc_serialplot.h"
// #include "1_Middleware/1_Driver/BSP/drv_djiboarda.h"
// #include "1_Middleware/1_Driver/TIM/drv_tim.h"
// #include "1_Middleware/1_Driver/WDG/drv_wdg.h"
// #include "Image_Transform/dvc_image_transform.h"
// #include "Vofa/dvc_Vofa.h"
//
// // 全局初始化完成标志位
// bool init_finished = false;
// uint32_t flag = 0;
// Class_Motor_DJI_C620 Motor_3508;
// volatile uint32_t a = 0;
//
// Class_Vofa_JustFloat Vofa_JustFloat;
// float VOFA_Target_Omega = 0.0f;
// float VOFA_Now_Omega = 0.0f;
// float VOFA_Target_Currnet = 0.0f;
// float VOFA_Now_Current = 0.0f;
//
// Class_Image_Transform vt13;
// static uint16_t cnt = 0;
// float x = 0.0f;
// float y = 0.0f;
// float dial = 0.0f;
//
// void Device_UART_Image_Transform_Callback(uint8_t *rx,uint16_t len)
// {
//     vt13.UART_RxCpltCallback(rx,len);
// }
//
// void Device_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
// {
//     switch (CAN_RxMessage->Header.StdId)
//     {
//     case(0x204):
//         Motor_3508.CAN_RxCpltCallback(CAN_RxMessage->Data);
//         break;
//
//     default:
//         break;
//     }
// }
//
// void Device_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
// {
//     switch (CAN_RxMessage->Header.StdId)
//     {
//     case (0x204):
//         Motor_3508.CAN_RxCpltCallback(CAN_RxMessage->Data);
//         break;
//
//     default:
//         break;
//     }
// }
// static void TIM_100ms_Callback()
// {
//     vt13.TIM_100ms_Alive_PeriodElapsedCallback();
//     if (vt13.Get_Status() == Image_Transform_Status_DISABLE)
//     {
//
//     }
// }
//
// static void TIM5_1ms_Callback()
// {
//     a++;
//
//     Motor_3508.TIM_Calculate_PeriodElapsedCallback();
//     TIM_1ms_CAN_PeriodElapsedCallback();
//
//     VOFA_Target_Omega = Motor_3508.Get_Target_Omega();
//     VOFA_Now_Omega = Motor_3508.Get_Now_Omega();
//     VOFA_Target_Currnet = Motor_3508.Get_Target_Current();
//
//     Vofa_JustFloat.TIM_1ms_PeriodElapsedCallback();
//
//
//     vt13.TIM_1ms_Calculate_PeriodElapsedCallback();
//     x = vt13.Get_Left_X()*80;
//     y = vt13.Get_Left_Y()*80;
//     dial = vt13.Get_Dial();
//     // Motor_3508.Set_Target_Omega(y);
//
//     if (++cnt >= 100)
//     {
//         cnt = 0;
//         TIM_100ms_Callback();
//     }
// }
//
//
//
// void Task_Init(void)
// {
//     CAN_Init(&hcan2,Device_CAN2_Callback);
//     Motor_3508.Init(&hcan2,Motor_DJI_ID_0x202,Motor_DJI_Control_Method_OMEGA);
//     //Motor_3508.Init(&hcan2,Motor_DJI_ID_0x201,Motor_DJI_Control_Method_CURRENT);
//
//     //Motor_3508.Init(&hcan2, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, 268.0f / 17.0f, Motor_DJI_Power_Limit_Status_ENABLE);
//     Motor_3508.PID_Omega.Init(0.3f,0.0f,0.0f,0.0f,3.0f,3.0f,0.001f);
//     //Motor_3508.Set_Target_Omega(0.0f);
//     Vofa_JustFloat.Init(&huart1,3,20);      //三个通道，20ms发一帧
//
//     Vofa_JustFloat.Bind_Channel(0,&VOFA_Target_Omega);
//     Vofa_JustFloat.Bind_Channel(1,&VOFA_Now_Omega);
//     Vofa_JustFloat.Bind_Channel(2,&VOFA_Target_Currnet);
//     //Vofa_JustFloat.Bind_Channel(3,&VOFA_Target_Currnet);
//
//     TIM_Init(&htim5, TIM5_1ms_Callback);
//     // HAL_TIM_Base_Start_IT(&htim5);
//
//     vt13.Init(&huart6);
//     UART_Init(&huart6,Device_UART_Image_Transform_Callback,64);
//
//     init_finished = true;
//
//     HAL_TIM_Base_Start_IT(&htim5);
// }
//
// void Task_Loop()
// {
//     // x = vt13.Get_Left_X()*80;
//     // y = vt13.Get_Left_Y()*80;
//     // dial = vt13.Get_Dial();
//     //Motor_3508.Set_Target_Omega(y);
//
//
//     // 直接手动构造一个 0x200 帧：只给第4个位置一个 3000 的电流，其余为0
//     // extern uint8_t CAN1_0x200_Tx_Data[8];
//     //
//     // int16_t cur1 = 1000;
//     // int16_t cur2 = 1000;
//     // int16_t cur3 = 1000;
//     // int16_t cur4 = 1000;  // 2~3A 左右，方向看极性
//     //
//     // CAN1_0x200_Tx_Data[0] = cur1 >> 8;
//     // CAN1_0x200_Tx_Data[1] = cur1 & 0xFF;
//     // CAN1_0x200_Tx_Data[2] = cur2 >> 8;
//     // CAN1_0x200_Tx_Data[3] = cur2 & 0xFF;
//     // CAN1_0x200_Tx_Data[4] = cur3 >> 8;9
//     // CAN1_0x200_Tx_Data[5] = cur3 & 0xFF;
//     // CAN1_0x200_Tx_Data[6] = cur4 >> 8;
//     // CAN1_0x200_Tx_Data[7] = cur4 & 0xFF;
//     //
//     // // 手动发一发
//     // CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);
// }
