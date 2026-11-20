/**
 * @file control.cpp
 * @brief Fixed-rate control input and foreground communications.
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
#include "incremental_pid.hpp"

namespace application
{
        using namespace control_config;

        /**
         * @brief Return the application instance shared by the interrupt and foreground loop.
         * @return The sole control instance.
         *
         */
        control& control::instance()
        {
                static control controller;
                return controller;
        }

        /**
         * @brief Receive newline-delimited VSET commands in millivolts.
         * @param uart Initialized UART used to receive commands.
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
                                if (!command_overflow && std::strncmp(command_line, "VSET ", 5) == 0 &&
                                        command_length > 5 && command_length <= 11 &&
                                        std::strspn(command_line + 5, "0123456789") == command_length - 5) {
                                        const auto requested = static_cast<uint32_t>(
                                                std::strtoul(command_line + 5, nullptr, 10));
                                        board::critical_section lock;
                                        target_mv = bus_undervoltage ? 0U : requested;
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
         * @brief Stop both bridges on bus undervoltage and discard the previous setpoint.
         * @param code Raw bus sample from PA7.
         *
         */
        void control::checkBusVoltage(uint16_t code)
        {
                const bool low = code < (bus_undervoltage ? bus_recover_code : bus_trip_code);
                if (low != bus_undervoltage) {
                        bus_undervoltage = low;
                        drivers::gpio::write(GPIOB, board::LedPower, low);
                        if (low) {
                                target_mv = 0;
                                commanded_duty_a = 0;
                                auto& pwm = drivers::hrtim::instance();
                                if (pwm.running()) {
                                        pwm.stop();
                                }
                        }
                }
        }

        /**
         * @brief Close the capacitor voltage loop using the A high-side duty.
         * @param voltage_code Raw capacitor voltage sample.
         * @param bus_code Raw bus voltage sample.
         *
         */
        void control::updateVoltageLoop(uint16_t voltage_code, uint16_t bus_code)
        {
                auto& pwm = drivers::hrtim::instance();
                const uint32_t requested = target_mv;
                if (requested == 0 || bus_code == 0 || bus_code >= adc_full_scale) {
                        if (pwm.running()) {
                                pwm.stop();
                        }
                        commanded_duty_a = 0;
                        return;
                }
                const float target = static_cast<float>(requested) * 0.001f;
                const float measured = voltage_code * capacitor_volts_per_code;
                if (pwm.faulted()) {
                        return;
                }
                const float bus = bus_code * capacitor_volts_per_code;
                const float feedforward = std::fmin(950.0f, std::fmax(50.0f, high_side_b * target / bus));
                const bool running = pwm.running();
                auto next = voltage_loop;
                if (!running) {
                        next.reset(feedforward, 0.0f);
                }
                const float change = running ? feedforward - previous_feedforward : 0.0f;
                const auto duty = static_cast<uint16_t>(next.update(target, measured, change) + 0.5f);
                const bool accepted = running ? (duty == commanded_duty_a || pwm.update(duty, high_side_b)) :
                                                        pwm.start(duty, high_side_b);
                if (accepted) {
                        voltage_loop = next;
                        previous_feedforward = feedforward;
                        commanded_duty_a = duty;
                }
        }
        /**
         * @brief Consume the next ADC window at the master timer interrupt.
         *
         */
        void control::handleControlTick()
        {
                if (!drivers::hrtim::takeControlTick() || control_timeout) {
                        return;
                }
                const uint32_t begin = DWT->CYCCNT;
                const uint32_t elapsed = begin - previous_tick;
                previous_tick = begin;
                if (windows != 0 && elapsed > period_cycles + period_cycles / 2U) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                        return;
                }
                if (windows != 0) {
                        const uint32_t error = elapsed >= period_cycles ?
                                elapsed - period_cycles : period_cycles - elapsed;
                        if (error > peak_period_error_cycles) {
                                peak_period_error_cycles = error;
                        }
                }
                ++windows;
                if (!drivers::adc::instance().readWindow(windows, input)) {
                        ++missed;
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
                        else if ((windows & (voltage_loop_divider - 1U)) == 0U) {
                                measured_mv = static_cast<uint16_t>(
                                        input.last.capacitor_voltage * capacitor_volts_per_code * 1000.0f + 0.5f);
                                updateVoltageLoop(input.last.capacitor_voltage, input.last.chassis_voltage);
                        }
                }
                latest_cycles = DWT->CYCCNT - begin;
                if (drivers::hrtim::controlTickPending()) {
                        control_timeout = true;
                        drivers::hrtim::disableOutputs();
                }
        }

        /**
         * @brief Arm both DMA receivers before starting the hardware window clock.
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
                                uint32_t sequence;
                                uint32_t lost;
                                uint32_t cycles;
                                uint32_t jitter;
                                uint32_t target;
                                unsigned voltage;
                                unsigned duty;
                                {
                                        board::critical_section lock;
                                        sequence = windows;
                                        lost = missed;
                                        cycles = latest_cycles;
                                        jitter = peak_period_error_cycles;
                                        peak_period_error_cycles = 0;
                                        target = target_mv;
                                        voltage = measured_mv;
                                        duty = drivers::hrtim::instance().faulted() ? 0U : commanded_duty_a;
                                }
                                clearerr(stdout);
                                std::printf("voltage:%lu,%u,%u,%lu,%lu,%lu,%lu,%u\n",
                                                        static_cast<unsigned long>(target), voltage, duty,
                                                        static_cast<unsigned long>(cycles),
                                                        static_cast<unsigned long>(jitter),
                                                        static_cast<unsigned long>(lost),
                                                        static_cast<unsigned long>(sequence),
                                                        static_cast<unsigned>(control_timeout));
                        }
                        __WFI();
                }
        }

        /**
         * @brief Enter the foreground loop for the application singleton.
         *
         */
        [[noreturn]] void run()
        {
                control::instance().run();
        }
}

/**
 * @brief Dispatch the HRTIM master interrupt to the control application.
 *
 */
extern "C" void HRTIM1_Master_IRQHandler()
{
        application::control::instance().handleControlTick();
}
