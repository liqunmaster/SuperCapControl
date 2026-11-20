/**
 * @file control.hpp
 * @brief Bare-metal control application and its runtime state.
 */
#pragma once

#include <cstddef>

#include "adc.hpp"
#include "board.hpp"
#include "incremental_pid.hpp"

namespace drivers
{
        class uart;
}

namespace application
{
        namespace control_config
        {
                constexpr uint32_t period_cycles = board::CoreHz / board::ControlHz;
                constexpr uint32_t heartbeat_half_period_ms = 500U;
                constexpr std::size_t telemetry_tx_low_watermark = 256U;
                constexpr uint16_t adc_full_scale = 4095U;
                constexpr unsigned bus_trip_mv = 18000U;
                constexpr unsigned bus_recover_mv = 20000U;
                constexpr unsigned bus_full_scale_mv = 33000U;
                constexpr unsigned bus_trip_code =
                        (bus_trip_mv * adc_full_scale + bus_full_scale_mv - 1U) / bus_full_scale_mv;
                constexpr unsigned bus_recover_code =
                        (bus_recover_mv * adc_full_scale + bus_full_scale_mv - 1U) / bus_full_scale_mv;
                constexpr float capacitor_volts_per_code = 33.0f / adc_full_scale;
                constexpr uint32_t voltage_loop_divider = 32U;
                constexpr float voltage_loop_period = static_cast<float>(voltage_loop_divider) / board::ControlHz;
                constexpr uint16_t high_side_b = 900U;
                constexpr math::pid_config voltage_pid{10.0f, 20.0f, 0.0f, voltage_loop_period, 50.0f, 950.0f};
        }

        /**
         * @brief Own the foreground command parser and interrupt-driven control state.
         *
         */
        class control
        {
                public:
                        static control& instance();
                        [[noreturn]] void run();
                        void handleControlTick();

                private:
                        void receiveVoltageTarget(drivers::uart& uart);
                        void checkBusVoltage(uint16_t code);
                        void updateVoltageLoop(uint16_t voltage_code, uint16_t bus_code);

                        math::incremental_pid voltage_loop{control_config::voltage_pid};
                        float previous_feedforward = 0.0f;
                        volatile uint32_t target_mv = 0;
                        volatile bool bus_undervoltage = true;
                        uint16_t measured_mv = 0;
                        uint16_t commanded_duty_a = 0;
                        volatile uint32_t windows = 0;
                        volatile uint32_t missed = 0;
                        volatile uint32_t latest_cycles = 0;
                        volatile uint32_t peak_period_error_cycles = 0;
                        volatile bool control_timeout = false;
                        uint32_t previous_tick = 0;
                        drivers::control_sample input{};
                        char stdout_buffer[256]{};
                        char command_line[32]{};
                        std::size_t command_length = 0;
                        bool command_overflow = false;
        };

        [[noreturn]] void run();
}
