/** @file board.cpp
 *  @brief Board clock, safe pin states and fatal fault handlers.
 */
#include "board.hpp"

#include "gpio.hpp"
#include "hrtim.hpp"

namespace board
{
    /** @brief Initialize safe pins and switch from HSI to the 170 MHz HSE PLL. */
    bool init()
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        drivers::gpio::output(GPIOA, PwmPins, false);
        drivers::gpio::output(GPIOB, LedGreen | LedBlue | LedRed | LedPower, false);

        SysTick->CTRL = 0;
        SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
        if (HAL_Init() != HAL_OK)
        {
            return false;
        }

        __HAL_RCC_PWR_CLK_ENABLE();
        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
        {
            return false;
        }

        RCC_OscInitTypeDef oscillator{};
        oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        oscillator.HSEState = RCC_HSE_ON;
        oscillator.PLL.PLLState = RCC_PLL_ON;
        oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        oscillator.PLL.PLLM = RCC_PLLM_DIV6;
        oscillator.PLL.PLLN = 85;
        oscillator.PLL.PLLP = RCC_PLLP_DIV2;
        oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
        oscillator.PLL.PLLR = RCC_PLLR_DIV2;
        if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
        {
            return false;
        }

        RCC_ClkInitTypeDef clock{};
        clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
        clock.APB1CLKDivider = RCC_HCLK_DIV1;
        clock.APB2CLKDivider = RCC_HCLK_DIV1;
        if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_4) != HAL_OK || SystemCoreClock != CoreHz)
        {
            return false;
        }

        __HAL_RCC_DMAMUX1_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        return true;
    }

    /** @brief Return the current APB1 timer input clock in Hz. */
    uint32_t apb1TimerHz()
    {
        const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        return (RCC->CFGR & RCC_CFGR_PPRE1) >= RCC_CFGR_PPRE1_DIV2 ? pclk * 2U : pclk;
    }

    /** @brief Force the PWM outputs low and halt after a fatal fault. */
    [[noreturn]] void panic()
    {
        __disable_irq();
        drivers::hrtim::disableOutputs();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        drivers::gpio::output(GPIOA, PwmPins, false);
        drivers::gpio::output(GPIOB, LedRed, true);
        while (true)
        {
            __NOP();
        }
    }
}

/** @brief Route a processor fault through the common shutdown. */
extern "C" void HardFault_Handler()
{
    board::panic();
}

/** @brief Route a memory protection fault through the common shutdown. */
extern "C" void MemManage_Handler()
{
    board::panic();
}

/** @brief Route a bus fault through the common shutdown. */
extern "C" void BusFault_Handler()
{
    board::panic();
}

/** @brief Route an instruction or processor state fault through the common shutdown. */
extern "C" void UsageFault_Handler()
{
    board::panic();
}
