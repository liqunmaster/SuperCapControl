/**
 * @brief 裸机启动与基础驱动验证入口。
 * @file main.cpp
 */
#include <cstdio>

#include "adc_dma.hpp"
#include "board.hpp"
#include "can.hpp"
#include "hrtim.hpp"
#include "uart_dma.hpp"

/**
 * @brief 初始化驱动并在前台输出原始采样数据。
 * @return 正常运行时不返回。
 */
int main()
{
    if (!board::init())
    {
        board::panic();
    }
    drivers::hrtim::init();
    if (!drivers::uart_dma::instance().init())
    {
        board::panic();
    }
    setvbuf(stdout, nullptr, _IONBF, 0);
    std::printf("SuperCap STM32G474CBT6: 144MHz, UART 115200 8N1, PWM disabled\r\n");
    if (!drivers::adc_dma::instance().init())
    {
        board::panic();
    }
    if (SUPERCAP_CAN_WIRING_CONFIRMED && !drivers::can::instance().init())
    {
        board::panic();
    }
    std::printf("CAN2.0 1Mbps: %s\r\n", SUPERCAP_CAN_WIRING_CONFIRMED ? "enabled" : "wiring confirmation required");
    uint32_t last_report = board::millis();
    bool led = false;
    while (true)
    {
        const uint32_t now = board::millis();
        if (static_cast<uint32_t>(now - last_report) >= 1000)
        {
            last_report = now;
            led = !led;
            board::setLed(board::LedRed, led);
            drivers::adc_sample sample{};
            if (drivers::adc_dma::instance().latest(sample))
            {
                std::printf(
                    "ADC raw #%lu: %u %u %u %u %u %u\r\n", static_cast<unsigned long>(sample.sequence),
                    static_cast<unsigned>(sample.capacitor_voltage), static_cast<unsigned>(sample.capacitor_current),
                    static_cast<unsigned>(sample.chassis_voltage), static_cast<unsigned>(sample.chassis_current),
                    static_cast<unsigned>(sample.output_voltage), static_cast<unsigned>(sample.output_current));
            }
            else
            {
                std::printf("ADC unavailable, errors=%lu\r\n",
                            static_cast<unsigned long>(drivers::adc_dma::instance().errors()));
            }
        }
        drivers::can_frame frame;
        for (unsigned budget = 0; budget < 15 && drivers::can::instance().receive(frame); ++budget)
        {
            board::setLed(board::LedBlue, true);
            /// No command protocol or control law has been selected yet.
        }
        __WFI();  ///< Tick and peripheral interrupts wake the foreground loop.
    }
}
