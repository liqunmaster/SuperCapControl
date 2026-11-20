/**
 * @file gpio.hpp
 * @brief HAL GPIO pin configuration and access.
 */
#pragma once

#include "stm32g4xx_hal.h"

namespace drivers
{
        /**
         * @brief Stateless GPIO access for an already clocked port.
         *
         */
        class gpio
        {
                public:
                        static void output(GPIO_TypeDef* port, uint16_t pins, bool initial);
                        static void analog(GPIO_TypeDef* port, uint16_t pins);
                        static void alternate(GPIO_TypeDef* port, uint16_t pins, uint32_t af,
                                              uint32_t pull = GPIO_NOPULL);
                        static void write(GPIO_TypeDef* port, uint16_t pins, bool high);
                        static bool read(GPIO_TypeDef* port, uint16_t pins);
                        static void toggle(GPIO_TypeDef* port, uint16_t pins);
        };
}
