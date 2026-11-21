/**
 * @file control.cpp
 * @brief 处理固定频率的控制与前台通信。
 */
#include "control.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "adc.hpp"
#include "board.hpp"
#include "board_options.hpp"
#include "can.hpp"
#include "gpio.hpp"
#include "hrtim.hpp"
#include "uart.hpp"

namespace application
{
        using namespace control_config;

        /**
         * @brief 获取控制中断和前台循环共享的应用实例。
         * @return 唯一的控制实例。
         *
         */
        control& control::instance()
        {
                static control controller;
                return controller;
        }

        /**
         * @brief 接收以换行符结束的十进制电压设定，单位为伏特。
         * @param uart 用于接收命令的已初始化串口。
         *
         */
        void control::receiveVoltageTarget(drivers::uart& uart)
        {
                for (unsigned count = 0; count < sizeof(command_line); ++count) {
                        uint8_t byte;
                        if (!uart.read(byte)) {
                                return;
                        }
                        if (byte == '\r' || byte == '\n') {
                                command_line[command_length] = '\0';
                                if (!command_overflow && command_length != 0 &&
                                        std::strspn(command_line, "0123456789.") == command_length) {
                                        char* end;
                                        const double millivolts = std::strtod(command_line, &end) * 1000.0;
                                        if (end == command_line + command_length && millivolts <= UINT32_MAX) {
                                                const auto requested = static_cast<uint32_t>(millivolts + 0.5);
                                                board::critical_section lock;
                                                target_mv      = requested;
                                                target_pending = !bus_undervoltage;
                                        }
                                }
                                command_length = 0;
                                command_overflow = false;
                        }
                        else if (command_length + 1 < sizeof(command_line)) {
                                command_line[command_length++] = static_cast<char>(byte);
                        }
                        else {
                                command_overflow = true;
                        }
                }
        }

        /**
         * @brief 母线欠压时关断两个桥臂，并取消待应用的 PWM 更新。
         * @param code 从 PA7 读取的母线电压 ADC 原始值。
         *
         */
        void control::checkBusVoltage(uint16_t code)
        {
                const bool low = code < (bus_undervoltage ? bus_recover_code : bus_trip_code);
                if (low != bus_undervoltage) {
                        bus_undervoltage = low;
                        drivers::gpio::write(GPIOB, board::LedPower, low);
                        if (low) {
                                target_pending = false;
                                commanded_duty_a = 0;
                                auto& pwm = drivers::hrtim::instance();
                                if (pwm.running()) {
                                        pwm.stop();
                                }
                        }
                }
        }

        /**
         * @brief 应用待处理的电容电压设定，并保持换算得到的占空比。
         * @param bus_code 母线电压的 ADC 原始值。
         *
         */
        void control::applyVoltageTarget(uint16_t bus_code)
        {
                auto& pwm = drivers::hrtim::instance();
                const uint32_t requested = target_mv;
                if (requested == 0 || bus_code == 0 || bus_code >= adc_full_scale) {
                        if (pwm.running()) {
                                pwm.stop();
                        }
                        commanded_duty_a = 0;
                        target_pending = false;
                        return;
                }
                if (!target_pending || pwm.faulted()) {
                        return;
                }
                const float bus       = bus_code * capacitor_volts_per_code;
                const float target    = static_cast<float>(requested) * 0.001f;
                const float estimated = high_side_b * target / bus;
                const auto duty = static_cast<uint16_t>(std::fmin(max_duty_a, std::fmax(min_duty_a, estimated)) + 0.5f);
                const bool accepted = pwm.running() ? (duty == commanded_duty_a || pwm.update(duty, high_side_b)) : pwm.start(duty, high_side_b);
                if (accepted) {
                        commanded_duty_a = duty;
                        target_pending = false;
                }
        }
        /**
         * @brief 在主定时器中断中读取并处理下一组 ADC 窗口采样。
         *
         */
        void control::handleControlTick()
        {
                if (!drivers::hrtim::takeControlTick() || control_timeout) {
                        return;
                }
                const uint32_t begin   = DWT->CYCCNT;
                const uint32_t elapsed = begin - previous_tick;
                previous_tick = begin;
                if (windows != 0 && elapsed > period_cycles + period_cycles / 2U) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                        return;
                }
                ++windows;
                if (!drivers::adc::instance().readWindow(windows, input)) {
                        commanded_duty_a = 0;
                }
                else {
                        checkBusVoltage(input.last.chassis_voltage);
                        if (input.last.capacitor_voltage >= adc_full_scale) {
                                measured_mv = 33000U;
                                commanded_duty_a = 0;
                                if (!drivers::hrtim::instance().faulted()) {
                                        drivers::hrtim::disableOutputs();
                                }
                        }
                        else if ((windows & (voltage_update_divider - 1U)) == 0U) {
                                measured_mv = static_cast<uint16_t>(
                                        input.last.capacitor_voltage * capacitor_volts_per_code * 1000.0f + 0.5f);
                                applyVoltageTarget(input.last.chassis_voltage);
                        }
                }
                if (drivers::hrtim::controlTickPending()) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                }
        }

        /**
         * @brief 启动硬件采样窗口定时器前，使能两路 ADC DMA 接收。
         *
         */
        [[noreturn]] void control::run()
        {
                if (!board::init()) {
                        board::panic();
                }
                drivers::gpio::write(GPIOB, board::LedPower, true);
                if (!drivers::hrtim::instance().init() || !drivers::uart::instance().init() ||
                        !drivers::adc::instance().init(drivers::hrtim::disableOutputs) ||
                        (board::can_wiring_confirmed && !drivers::can::instance().init()) ||
                        !drivers::hrtim::instance().startTimebase()) {
                        board::panic();
                }
                setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
                uint32_t heartbeat = HAL_GetTick();
                drivers::gpio::write(GPIOB, board::LedGreen, true);
                while (true) {
                        const uint32_t now = HAL_GetTick();
                        if (now - heartbeat >= heartbeat_half_period_ms) {
                                heartbeat = now;
                                drivers::gpio::toggle(GPIOB, board::LedGreen);
                        }
                        drivers::uart::instance().service();
                        receiveVoltageTarget(drivers::uart::instance());
                        drivers::can_frame frame{};
                        for (unsigned count = 0; count < 15 && drivers::can::instance().receive(frame); ++count) {
                                drivers::gpio::write(GPIOB, board::LedBlue, true);
                        }
                        if (drivers::uart::instance().txPending() < telemetry_tx_low_watermark) {
                                uint32_t target;
                                unsigned voltage;
                                unsigned duty;
                                {
                                        board::critical_section lock;
                                        target = target_mv;
                                        voltage = measured_mv;
                                        duty = drivers::hrtim::instance().faulted() ? 0U : commanded_duty_a;
                                }
                                clearerr(stdout);
                                std::printf("voltage:%lu.%03lu,%u.%03u,%u\n", static_cast<unsigned long>(target / 1000U), static_cast<unsigned long>(target % 1000U), voltage / 1000U, voltage % 1000U, duty);
                        }
                        __WFI();
                }
        }

        /**
         * @brief 进入控制应用实例的前台循环。
         *
         */
        [[noreturn]] void run()
        {
                control::instance().run();
        }
}

/**
 * @brief 将 HRTIM 主定时器中断交给控制应用处理。
 *
 */
extern "C" void HRTIM1_Master_IRQHandler()
{
        application::control::instance().handleControlTick();
}
