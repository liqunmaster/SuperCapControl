/** @file timebase.cpp
 * @brief TIM2 HAL milliseconds.
 */
#include "board.hpp"

/** @brief Configure HAL timeouts from TIM2 at boot and after clock changes. */
extern "C" HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    if (priority >= (1U << __NVIC_PRIO_BITS))
    {
        return HAL_ERROR;
    }
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = (board::apb1TimerHz() / 1000U) * static_cast<uint32_t>(uwTickFreq) - 1U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, priority, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_CEN;
    uwTickPrio = priority;
    return HAL_OK;
}

/** @brief Pause the HAL timebase interrupt. */
extern "C" void HAL_SuspendTick()
{
    TIM2->DIER &= ~TIM_DIER_UIE;
}

/** @brief Resume the HAL timebase interrupt. */
extern "C" void HAL_ResumeTick()
{
    TIM2->DIER |= TIM_DIER_UIE;
}

/** @brief Advance the HAL timebase once per TIM2 update. */
extern "C" void TIM2_IRQHandler()
{
    if ((TIM2->SR & TIM_SR_UIF) != 0)
    {
        TIM2->SR = 0;
        HAL_IncTick();
    }
}

