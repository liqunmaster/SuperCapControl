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
         * @brief 跨串口读取保存当前命令行。
         *
         */
        class voltage_command_reader
        {
                public:
                        bool read(drivers::uart& uart, uint32_t& target);

                private:
                        char command_line[32]{};
                        std::size_t command_length = 0;
                        bool command_overflow = false;
        };

        /**
         * @brief 接收以换行符结束的十进制电压设定，单位为伏特。
         * @param uart 用于接收命令的已初始化串口。
         *
         */
        bool voltage_command_reader::read(drivers::uart& uart, uint32_t& target)
        {
                bool received = false;
                for (unsigned count = 0; count < sizeof(command_line); ++count) {
                        uint8_t byte;
                        if (!uart.read(byte)) {
                                break;
                        }
                        if (byte == '\r' || byte == '\n') {
                                command_line[command_length] = '\0';
                                if (!command_overflow && command_length != 0 && std::strspn(command_line, "0123456789.") == command_length) {
                                        char* end;
                                        const double millivolts = std::strtod(command_line, &end) * 1000.0;
                                        if (end == command_line + command_length && millivolts <= UINT32_MAX) {
                                                target = static_cast<uint32_t>(millivolts + 0.5);
                                                received = true;
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
                return received;
        }

        /**
         * @brief 根据母线电压更新欠压指示灯。
         * @param code 从 PA7 读取的母线电压 ADC 原始值。
         *
         */
        void control::checkBusVoltage(uint16_t code)
        {
                constexpr unsigned bus_trip_mv      = 18000U;
                constexpr unsigned bus_recover_mv   = 20000U;
                constexpr unsigned bus_trip_code    = (bus_trip_mv * adc_full_scale + voltage_full_scale_mv - 1U) / voltage_full_scale_mv;
                constexpr unsigned bus_recover_code = (bus_recover_mv * adc_full_scale + voltage_full_scale_mv - 1U) / voltage_full_scale_mv;
                const bool low = code < (bus_undervoltage ? bus_recover_code : bus_trip_code);
                if (low != bus_undervoltage) {
                        bus_undervoltage = low;
                        drivers::gpio::write(GPIOB, board::LedPower, low);
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
                constexpr uint32_t period_cycles = board::pwm_timing::control_cpu_cycles;
                if (windows != 0 && elapsed > period_cycles + period_cycles / 2U) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                        return;
                }
                ++windows;
                drivers::control_sample input{};
                if (drivers::adc::instance().readWindow(windows, input)) {
                        checkBusVoltage(input.last.chassis_voltage);
                        if (input.last.capacitor_voltage >= adc_full_scale) {
                                measured_mv = voltage_full_scale_mv;
                                measured_capacitor_code = adc_full_scale;
                                if (!drivers::hrtim::instance().faulted()) {
                                        drivers::hrtim::disableOutputs();
                                }
                        }
                        else if ((windows & (voltage_loop::voltage_update_divider - 1U)) == 0U) {
                                measured_capacitor_code = input.last.capacitor_voltage;
                                measured_mv = static_cast<uint16_t>(measured_capacitor_code * voltage_mv_per_code + 0.5f);
                                if ((target_mv == 0) || (input.last.chassis_voltage >= adc_full_scale)) {
                                        regulator.stop();
                                }
                                else {
                                        constexpr float scope_bus_reference_v = 24.0f;
                                        const float bus = bus_undervoltage ? scope_bus_reference_v : input.last.chassis_voltage * voltage_mv_per_code * 0.001f;
                                        regulator.apply(target_mv, bus, measured_capacitor_code * voltage_mv_per_code * 0.001f, bus_undervoltage);
                                }
                        }
                }
                if (drivers::hrtim::controlTickPending()) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                }
        }

        /**
         * @brief 初始化板级设备，并在前台处理串口命令和遥测。
         *
         */
        void control::run()
        {
                if (!board::init()) {
                        board::panic();
                }
                drivers::gpio::write(GPIOB, board::LedPower, true);
                auto& pwm  = drivers::hrtim::instance();
                auto& uart = drivers::uart::instance();
                auto& adc  = drivers::adc::instance();
                if (!pwm.init()) {
                        board::panic();
                }
                if (!uart.init()) {
                        board::panic();
                }
                if (!adc.init(drivers::hrtim::disableOutputs)) {
                        board::panic();
                }
                if (board::can_wiring_confirmed && !drivers::can::instance().init()) {
                        board::panic();
                }
                // ADC 和 DMA 就绪后，才能启动采样触发。
                if (!pwm.startTimebase()) {
                        board::panic();
                }
                setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
                voltage_command_reader reader;
                uint32_t heartbeat = HAL_GetTick();
                drivers::gpio::write(GPIOB, board::LedGreen, true);
                while (true) {
                        const uint32_t now = HAL_GetTick();
                        constexpr uint32_t heartbeat_half_period_ms = 500U;
                        if (now - heartbeat >= heartbeat_half_period_ms) {
                                heartbeat = now;
                                drivers::gpio::toggle(GPIOB, board::LedGreen);
                        }
                        uart.service();
                        uint32_t requested;
                        if (reader.read(uart, requested)) {
                                board::critical_section lock;
                                target_mv = requested;
                        }
                        drivers::can_frame frame{};
                        for (unsigned count = 0; count < 15 && drivers::can::instance().receive(frame); ++count) {
                                drivers::gpio::write(GPIOB, board::LedBlue, true);
                        }
                        constexpr std::size_t telemetry_tx_low_watermark = 256U;
                        if (uart.txPending() < telemetry_tx_low_watermark) {
                                uint32_t target;
                                unsigned voltage;
                                unsigned capacitor_code;
                                {
                                        board::critical_section lock;
                                        target = target_mv;
                                        voltage = measured_mv;
                                        capacitor_code = measured_capacitor_code;
                                }
                                const unsigned pin_voltage_100uv = (capacitor_code * adc_reference_mv * 10U + adc_full_scale / 2U) / adc_full_scale;
                                clearerr(stdout);
                                std::printf("%lu.%03lu,%u.%03u,%u.%04u\n", static_cast<unsigned long>(target / 1000U), static_cast<unsigned long>(target % 1000U), voltage / 1000U, voltage % 1000U, pin_voltage_100uv / 10000U, pin_voltage_100uv % 10000U);
                        }
                        __WFI();
                }
        }

        /**
         * @brief 进入控制应用实例的前台循环。
         *
         */
        void run()
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
