#ifndef C_BOARD_CLASS_TIMER_H
#define C_BOARD_CLASS_TIMER_H

#include "stm32h7xx_hal.h"

class User_Timer
{
public:
    User_Timer();

    void Get_Dt_Result();

    float dt;

private:
    static void DWT_Init();
    static uint32_t Get_Cycle_Count();
    static uint32_t Get_Elapsed_Cycles(uint32_t last_cycle_count, uint32_t now_cycle_count);
    static float Convert_Cycle_To_Second(uint32_t cycle_count);

private:
    uint32_t last_cycle_count;
    bool has_last_timepoint;

    static bool dwt_is_ready;
};

#endif // C_BOARD_CLASS_TIMER_H
