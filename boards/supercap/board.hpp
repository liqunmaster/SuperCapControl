/**
 * @brief 板级初始化与基础工具接口。
 * @file board.hpp
 */
#pragma once
#include <cstdint>

#include "stm32g4xx.h"

/// @brief 板级资源与初始化工具。

namespace board
{
    constexpr uint32_t HseHz = 24000000;    ///< 外部晶振频率，单位 Hz。
    constexpr uint32_t CoreHz = 144000000;  ///< 配置后的内核及总线频率，单位 Hz。
    constexpr uint32_t UartBaud = 115200;   ///< 调试串口波特率。
    constexpr uint32_t SamplingHz = 1000;   ///< Bring-up only; not the control loop frequency.
    constexpr unsigned LedGreen = 12;       ///< 绿色 LED 的 GPIOB 引脚。
    constexpr unsigned LedBlue = 13;        ///< 蓝色 LED 的 GPIOB 引脚。
    constexpr unsigned LedRed = 14;         ///< 红色 LED 的 GPIOB 引脚。
    constexpr unsigned LedPower = 15;       ///< 电源指示 LED 的 GPIOB 引脚。

    /**
     * @brief 保存并恢复 PRIMASK 的临界区守卫，支持嵌套使用。
     */
    class critical_section
    {
        public:
            /**
             * @brief 保存当前中断屏蔽状态并屏蔽可屏蔽中断。
             */
            critical_section() : _mask(__get_PRIMASK())
            {
                __disable_irq();
            }
            /**
             * @brief 恢复进入临界区前的 PRIMASK。
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
    bool waitBits(volatile uint32_t& reg, uint32_t mask, uint32_t value);
    void delayUs(uint32_t us);
    void pinMode(GPIO_TypeDef* port, unsigned pin, unsigned mode, unsigned af = 0);
    void setLed(unsigned pin, bool on);
    uint32_t millis();
    [[noreturn]] void panic();
}
