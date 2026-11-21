/**
 * @file board.hpp
 * @brief Board clock, pin assignments and short interrupt guard.
 */
#pragma once

#include <cstdint>

#include "stm32g4xx_hal.h"

namespace board
{
        constexpr uint32_t CoreHz = 170000000;       ///< HSE/PLL and AHB frequency.
        constexpr uint32_t ControlHz = 31250;       ///< Control windows per second.
        constexpr uint32_t UartBaud = 115200;        ///< USART3 line rate.
        constexpr uint32_t PwmHz = 125000;          ///< HRTIM bench switching frequency.
        constexpr uint32_t PwmDeadTimeNs = 100;     ///< Bench software dead time per edge.
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
         * @brief Short, nestable PRIMASK guard for task and IRQ data handoff.
         *
         */
        class critical_section
        {
                public:
                        /**
                         * @brief Save PRIMASK and mask configurable interrupts.
                         *
                         */
                        critical_section() : _mask(__get_PRIMASK())
                        {
                                __disable_irq();
                        }

                        /**
                         * @brief Restore the caller's PRIMASK.
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
