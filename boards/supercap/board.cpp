/**
 * @file board.cpp
 * @brief Board clock, safe pin states and fatal fault handlers.
 */
#include "board.hpp"

namespace
{
        /**
         * @brief Set the output latch before switching the pins to output mode.
         *
         */
        void safeOutput(GPIO_TypeDef* port, uint32_t pins, GPIO_PinState level)
        {
                HAL_GPIO_WritePin(port, static_cast<uint16_t>(pins), level);
                GPIO_InitTypeDef config{};
                config.Pin = pins;
                config.Mode = GPIO_MODE_OUTPUT_PP;
                config.Speed = GPIO_SPEED_FREQ_HIGH;
                HAL_GPIO_Init(port, &config);
        }
}

namespace board
{
        /**
         * @brief Initialize safe pins and switch from HSI to the 170 MHz HSE PLL.
         *
         */
        bool init()
        {
                __HAL_RCC_GPIOA_CLK_ENABLE();
                __HAL_RCC_GPIOB_CLK_ENABLE();
                safeOutput(GPIOA, PwmPins, GPIO_PIN_RESET);
                safeOutput(GPIOB, LedGreen | LedBlue | LedRed | LedPower, GPIO_PIN_RESET);

                SysTick->CTRL = 0;
                SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
                if (HAL_Init() != HAL_OK) {
                        return false;
                }

                __HAL_RCC_PWR_CLK_ENABLE();
                if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK) {
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
                if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
                        return false;
                }

                RCC_ClkInitTypeDef clock{};
                clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
                clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
                clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
                clock.APB1CLKDivider = RCC_HCLK_DIV1;
                clock.APB2CLKDivider = RCC_HCLK_DIV1;
                if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_4) != HAL_OK || SystemCoreClock != CoreHz) {
                        return false;
                }

                __HAL_RCC_DMAMUX1_CLK_ENABLE();
                __HAL_RCC_DMA1_CLK_ENABLE();
                CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
                DWT->CYCCNT = 0;
                DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
                return true;
        }

        /**
         * @brief Return the current APB1 timer input clock in Hz.
         *
         */
        uint32_t apb1TimerHz()
        {
                const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
                return (RCC->CFGR & RCC_CFGR_PPRE1) >= RCC_CFGR_PPRE1_DIV2 ? pclk * 2U : pclk;
        }

        /**
         * @brief Force the PWM outputs low and halt after a fatal fault.
         *
         */
        [[noreturn]] void panic()
        {
                __disable_irq();
                if (__HAL_RCC_HRTIM1_IS_CLK_ENABLED()) {
                        HRTIM1->sCommonRegs.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS |
                                                              HRTIM_ODISR_TB1ODIS | HRTIM_ODISR_TB2ODIS;
                }
                __HAL_RCC_GPIOA_CLK_ENABLE();
                __HAL_RCC_GPIOB_CLK_ENABLE();
                safeOutput(GPIOA, PwmPins, GPIO_PIN_RESET);
                safeOutput(GPIOB, LedRed, GPIO_PIN_SET);
                while (true) {
                        __NOP();
                }
        }
}

/**
 * @brief Route a processor fault through the common shutdown.
 *
 */
extern "C" void HardFault_Handler()
{
        board::panic();
}

/**
 * @brief Route a memory protection fault through the common shutdown.
 *
 */
extern "C" void MemManage_Handler()
{
        board::panic();
}

/**
 * @brief Route a bus fault through the common shutdown.
 *
 */
extern "C" void BusFault_Handler()
{
        board::panic();
}

/**
 * @brief Route an instruction or processor state fault through the common shutdown.
 *
 */
extern "C" void UsageFault_Handler()
{
        board::panic();
}
