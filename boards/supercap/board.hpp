/**
 * @file board.hpp
 * @brief 板级时钟、引脚分配与短临界区。
 */
#pragma once

#include <cstdint>

#include "stm32g4xx_hal.h"

namespace board
{
        /**
         * @brief HSE 经 PLL 倍频后的内核与 AHB 时钟频率，单位为 Hz。
         *
         */
        constexpr uint32_t CoreHz = 170000000;
        /**
         * @brief USART3 的波特率。
         *
         */
        constexpr uint32_t UartBaud = 115200;
        /**
         * @brief 两侧 PWM 的目标开关频率，单位为 Hz。硬件周期取最接近的整数计数。
         *
         */
        constexpr uint32_t PwmHz = 125000;
        /**
         * @brief PWM 计数与控制窗口共用的时基。主定时器按 ×8 计数，桥臂定时器按 ×32 计数。
         *
         */
        struct pwm_timing
        {
                static constexpr uint32_t bridge_multiplier = 32U;
                static constexpr uint32_t master_multiplier = 8U;
                static constexpr uint32_t cycles_per_window = bridge_multiplier / master_multiplier;
                static constexpr uint64_t timer_hz          = static_cast<uint64_t>(CoreHz) * bridge_multiplier;
                static constexpr uint32_t period_ticks      = static_cast<uint32_t>((timer_hz + PwmHz / 2U) / PwmHz);
                static constexpr uint32_t control_cpu_cycles = (period_ticks + master_multiplier - 1U) / master_multiplier;
        };
        /**
         * @brief HRTIM 对每个换向边沿插入的死区，单位为 ns。
         *
         */
        constexpr uint32_t PwmDeadTimeNs = 100;
        /**
         * @brief 两路 ADC 每个 PWM 周期扫描一次。时间以半个 ADC 时钟为单位。
         *
         */
        struct adc_timing
        {
                static constexpr unsigned clock_divider        = 4;
                static constexpr unsigned acquisition_halves   = 95;
                static constexpr unsigned conversion_halves    = 25;
                static constexpr unsigned scans_per_window     = pwm_timing::cycles_per_window;
                static constexpr unsigned adc1_channels        = 2;
                static constexpr unsigned adc2_channels        = 4;
                static constexpr unsigned capacitor_current_index = 2;
                static constexpr auto prescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
                static constexpr auto sample_time = ADC_SAMPLETIME_47CYCLES_5;
        };
        constexpr unsigned AdcErrorPriority   = 2;
        constexpr unsigned AdcTriggerPriority = 3;
        constexpr unsigned AdcDmaPriority     = 4;
        constexpr unsigned PwmDmaPriority     = 3;
        constexpr unsigned CanPriority        = 8;
        constexpr unsigned UartPriority       = 10;

        constexpr uint16_t LedGreen = GPIO_PIN_12;
        constexpr uint16_t LedBlue  = GPIO_PIN_13;
        constexpr uint16_t LedRed   = GPIO_PIN_14;
        constexpr uint16_t LedPower = GPIO_PIN_15;
        constexpr uint16_t PwmPins  = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;

        /**
         * @brief 使用 PRIMASK 保护前台与中断之间的数据交接，支持嵌套。
         *
         */
        class critical_section
        {
                public:
                        /**
                         * @brief 保存 PRIMASK 并屏蔽可配置优先级的中断。
                         *
                         */
                        critical_section() : _mask(__get_PRIMASK())
                        {
                                __disable_irq();
                        }

                        /**
                         * @brief 恢复进入临界区前的 PRIMASK。
                         *
                         */
                        ~critical_section()
                        {
                                __set_PRIMASK(_mask);
                        }

                        critical_section(const critical_section&) = delete;
                        critical_section& operator=(const critical_section&) = delete;

                private:
                        uint32_t _mask;
        };

        bool init();
        uint32_t apb1TimerHz();
        [[noreturn]] void panic();
}
