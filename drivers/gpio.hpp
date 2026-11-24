/**
 * @file gpio.hpp
 * @brief GPIO 引脚配置与读写接口。
 */
#pragma once

#include "stm32g4xx_hal.h"

namespace drivers
{
        /**
         * @brief 提供已开启时钟的 GPIO 端口访问函数。
         *
         */
        class gpio
        {
                public:
                        static void output(GPIO_TypeDef* port, uint16_t pins, bool initial);
                        static void analog(GPIO_TypeDef* port, uint16_t pins);
                        static void alternate(GPIO_TypeDef* port, uint16_t pins, uint32_t af, uint32_t pull = GPIO_NOPULL);
                        static void write(GPIO_TypeDef* port, uint16_t pins, bool high);
                        static bool read(GPIO_TypeDef* port, uint16_t pins);
                        static void toggle(GPIO_TypeDef* port, uint16_t pins);
        };
}
