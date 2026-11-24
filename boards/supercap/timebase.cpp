/**
 * @file timebase.cpp
 * @brief 由 TIM2 提供 HAL 毫秒时基。
 */
#include "board.hpp"

/**
 * @brief 在启动和时钟切换后配置 TIM2，使 HAL 超时计数保持毫秒分辨率。
 *
 */
extern "C" HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
        if (priority >= (1U << __NVIC_PRIO_BITS)) {
                return HAL_ERROR;
        }
        __HAL_RCC_TIM2_CLK_ENABLE();
        TIM2->CR1 = 0;
        TIM2->PSC = 0;
        TIM2->ARR = (board::apb1TimerHz() / 1000U) * static_cast<uint32_t>(uwTickFreq) - 1U;
        TIM2->EGR = TIM_EGR_UG;
        TIM2->SR  = 0;
        HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
        HAL_NVIC_SetPriority(TIM2_IRQn, priority, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        TIM2->DIER = TIM_DIER_UIE;
        TIM2->CR1  = TIM_CR1_CEN;
        uwTickPrio = priority;
        return HAL_OK;
}

/**
 * @brief 暂停 HAL 时基中断。
 *
 */
extern "C" void HAL_SuspendTick()
{
        TIM2->DIER &= ~TIM_DIER_UIE;
}

/**
 * @brief 恢复 HAL 时基中断。
 *
 */
extern "C" void HAL_ResumeTick()
{
        TIM2->DIER |= TIM_DIER_UIE;
}

/**
 * @brief 每次 TIM2 更新时递增 HAL 毫秒计数。
 *
 */
extern "C" void TIM2_IRQHandler()
{
        if ((TIM2->SR & TIM_SR_UIF) != 0) {
                TIM2->SR = 0;
                HAL_IncTick();
        }
}

