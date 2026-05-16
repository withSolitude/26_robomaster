#include "class_timer.h"

bool User_Timer::dwt_is_ready = false;

User_Timer::User_Timer()
{
    DWT_Init();
    last_cycle_count = 0U;
    has_last_timepoint = false;
    dt = 0.0f;
}

void User_Timer::DWT_Init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if (__CORTEX_M == 7U)
    DWT->LAR = 0xC5ACCE55UL;
#endif

    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U)
    {
        return;
    }

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_is_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}

uint32_t User_Timer::Get_Cycle_Count()
{
    if (dwt_is_ready == false)
    {
        DWT_Init();
        return 0U;
    }

    return DWT->CYCCNT;
}

uint32_t User_Timer::Get_Elapsed_Cycles(uint32_t last_cycle_count, uint32_t now_cycle_count)
{
    if (now_cycle_count >= last_cycle_count)
    {
        return now_cycle_count - last_cycle_count;
    }

    // DWT 为 32 位计数器，回绕后补上剩余区间。
    return (0xFFFFFFFFUL - last_cycle_count) + now_cycle_count + 1U;
}

float User_Timer::Convert_Cycle_To_Second(uint32_t cycle_count)
{
    uint32_t core_clock_hz = SystemCoreClock;
    return static_cast<float>(cycle_count) / static_cast<float>(core_clock_hz);
}

void User_Timer::Get_Dt_Result()
{
    uint32_t now_cycle_count = Get_Cycle_Count();

    if (dwt_is_ready == false)
    {
        has_last_timepoint = false;
        last_cycle_count = 0U;
        dt = 0.0f;
        return;
    }

    if (has_last_timepoint == false)
    {
        last_cycle_count = now_cycle_count;
        has_last_timepoint = true;
        dt = 0.0f;
        return;
    }

    dt = Convert_Cycle_To_Second(Get_Elapsed_Cycles(last_cycle_count, now_cycle_count));
    last_cycle_count = now_cycle_count;
}
