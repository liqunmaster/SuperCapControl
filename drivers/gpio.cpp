/**
 * @file gpio.cpp
 * @brief GPIO 引脚配置与读写。
 */
#include "gpio.hpp"

#include "board.hpp"

namespace
{
        /**
         * @brief 配置已开启端口时钟的一个或多个 GPIO 引脚。
         *
         */
        void configure(GPIO_TypeDef* port, uint16_t pins, uint32_t mode, uint32_t pull, uint32_t af)
        {
                GPIO_InitTypeDef config{};
                config.Pin       = pins;
                config.Mode      = mode;
                config.Pull      = pull;
                config.Speed     = GPIO_SPEED_FREQ_HIGH;
                config.Alternate = af;
                HAL_GPIO_Init(port, &config);
        }
}

namespace drivers
{
        /**
         * @brief 先设置初始电平，再切换至推挽输出模式。
         *
         */
        void gpio::output(GPIO_TypeDef* port, uint16_t pins, bool initial)
        {
                write(port, pins, initial);
                configure(port, pins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
        }

        /**
         * @brief 选择模拟输入模式，关闭内部上下拉。
         *
         */
        void gpio::analog(GPIO_TypeDef* port, uint16_t pins)
        {
                configure(port, pins, GPIO_MODE_ANALOG, GPIO_NOPULL, 0);
        }

        /**
         * @brief 为指定引脚选择复用功能。
         *
         */
        void gpio::alternate(GPIO_TypeDef* port, uint16_t pins, uint32_t af, uint32_t pull)
        {
                configure(port, pins, GPIO_MODE_AF_PP, pull, af);
        }

        /**
         * @brief 通过 GPIO 置位复位寄存器一次更新掩码选中的引脚。
         *
         */
        void gpio::write(GPIO_TypeDef* port, uint16_t pins, bool high)
        {
                HAL_GPIO_WritePin(port, pins, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        /**
         * @brief 掩码中至少有一个引脚读为高电平时返回 true。
         *
         */
        bool gpio::read(GPIO_TypeDef* port, uint16_t pins)
        {
                return HAL_GPIO_ReadPin(port, pins) == GPIO_PIN_SET;
        }

        /**
         * @brief 在本地中断保护下翻转指定输出引脚。
         *
         */
        void gpio::toggle(GPIO_TypeDef* port, uint16_t pins)
        {
                board::critical_section lock;
                HAL_GPIO_TogglePin(port, pins);
        }
}
