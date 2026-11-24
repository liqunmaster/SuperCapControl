/**
 * @file board.cpp
 * @brief 板级时钟、引脚初始电平与致命故障处理。
 */
#include "board.hpp"

/**
 * @brief 在初始化时钟和运行环境前拉低两侧栅极驱动器的输入。
 * @pre 复位处理程序已设置栈指针。
 *
 */
extern "C" void board_early_gate_inputs_low()
{
        static_assert(board::PwmPins == 0x0f00U, "Update the GPIO mode masks when the gate pins move");
        constexpr uint32_t mode_mask   = GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11;
        constexpr uint32_t output_mode = GPIO_MODER_MODE8_0 | GPIO_MODER_MODE9_0 |
                                         GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0;
        constexpr uint32_t pull_down   = GPIO_PUPDR_PUPD8_1 | GPIO_PUPDR_PUPD9_1 |
                                         GPIO_PUPDR_PUPD10_1 | GPIO_PUPDR_PUPD11_1;
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIOA->BSRR = static_cast<uint32_t>(board::PwmPins) << 16U;
        GPIOA->OTYPER &= ~static_cast<uint32_t>(board::PwmPins);
        MODIFY_REG(GPIOA->PUPDR, mode_mask, pull_down);
        MODIFY_REG(GPIOA->MODER, mode_mask, output_mode);
        __DSB();
}

namespace
{
        /**
         * @brief 先设置输出锁存电平，再将引脚切换为输出模式。
         *
         */
        void safeOutput(GPIO_TypeDef* port, uint32_t pins, GPIO_PinState level)
        {
                HAL_GPIO_WritePin(port, static_cast<uint16_t>(pins), level);
                GPIO_InitTypeDef config{};
                config.Pin   = pins;
                config.Mode  = GPIO_MODE_OUTPUT_PP;
                config.Speed = GPIO_SPEED_FREQ_HIGH;
                HAL_GPIO_Init(port, &config);
        }
}

namespace board
{
        /**
         * @brief 设置引脚初始状态，并从 HSI 切换到由 HSE 经 PLL 产生的 170 MHz 系统时钟。
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
                oscillator.HSEState       = RCC_HSE_ON;
                oscillator.PLL.PLLState   = RCC_PLL_ON;
                oscillator.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
                oscillator.PLL.PLLM       = RCC_PLLM_DIV6;
                oscillator.PLL.PLLN       = 85;
                oscillator.PLL.PLLP       = RCC_PLLP_DIV2;
                oscillator.PLL.PLLQ       = RCC_PLLQ_DIV2;
                oscillator.PLL.PLLR       = RCC_PLLR_DIV2;
                if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
                        return false;
                }

                RCC_ClkInitTypeDef clock{};
                clock.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
                clock.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
                clock.AHBCLKDivider  = RCC_SYSCLK_DIV1;
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
         * @brief 返回 APB1 定时器的当前输入时钟频率，单位为 Hz。
         *
         */
        uint32_t apb1TimerHz()
        {
                const uint32_t pclk = HAL_RCC_GetPCLK1Freq();
                return (RCC->CFGR & RCC_CFGR_PPRE1) >= RCC_CFGR_PPRE1_DIV2 ? pclk * 2U : pclk;
        }

        /**
         * @brief 发生致命故障后拉低 PWM 输出并停机。
         *
         */
        [[noreturn]] void panic()
        {
                __disable_irq();
                if (__HAL_RCC_HRTIM1_IS_CLK_ENABLED()) {
                        HRTIM1->sCommonRegs.ODISR = HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS | HRTIM_ODISR_TB1ODIS | HRTIM_ODISR_TB2ODIS;
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
 * @brief 处理处理器硬故障并关断输出。
 *
 */
extern "C" void HardFault_Handler()
{
        board::panic();
}

/**
 * @brief 处理存储保护故障并关断输出。
 *
 */
extern "C" void MemManage_Handler()
{
        board::panic();
}

/**
 * @brief 处理总线故障并关断输出。
 *
 */
extern "C" void BusFault_Handler()
{
        board::panic();
}

/**
 * @brief 处理指令或处理器状态故障并关断输出。
 *
 */
extern "C" void UsageFault_Handler()
{
        board::panic();
}
