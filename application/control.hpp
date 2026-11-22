/**
 * @file control.hpp
 * @brief 裸机控制应用及其运行状态。
 */
#pragma once

#include <cstddef>

#include "adc.hpp"
#include "board.hpp"

namespace drivers
{
        class uart;
}

namespace application
{
        namespace control_config
        {
                constexpr uint32_t period_cycles                 = board::CoreHz / board::ControlHz;
                constexpr uint32_t heartbeat_half_period_ms      = 500U;
                constexpr std::size_t telemetry_tx_low_watermark = 256U;
                constexpr uint16_t adc_full_scale                = 4095U;
                constexpr unsigned bus_trip_mv                   = 18000U;
                constexpr unsigned bus_recover_mv                = 20000U;
                constexpr uint16_t adc_reference_mv              = 3000U;
                constexpr unsigned voltage_divider_gain          = (100U + 10U) / 10U;
                constexpr unsigned voltage_full_scale_mv        = voltage_divider_gain * adc_reference_mv;
                constexpr unsigned bus_trip_code                 = (bus_trip_mv * adc_full_scale + voltage_full_scale_mv - 1U) / voltage_full_scale_mv;
                constexpr unsigned bus_recover_code              = (bus_recover_mv * adc_full_scale + voltage_full_scale_mv - 1U) / voltage_full_scale_mv;
                /**
                 * @brief 母线欠压时用于换算示波器测量占空比的参考电压，单位为伏特。
                 *
                 */
                constexpr float scope_bus_reference_v     = 24.0f;
                constexpr float voltage_mv_per_code = static_cast<float>(voltage_full_scale_mv) / adc_full_scale;
                constexpr uint32_t voltage_update_divider = 32U;
                constexpr float min_duty                  = 50.0f;
                constexpr float max_duty                  = 950.0f;
                constexpr float buck_ratio                = 0.8f;
                constexpr float boost_ratio               = 1.25f;
                constexpr float transition_scale          = 400.0f;
                constexpr float fixed_duty                = transition_scale * (1.0f + boost_ratio);
        }

        /**
         * @brief 封装前台命令解析与中断驱动的控制状态。
         *
         */
        class control
        {
                public:
                        static control& instance();
                        void run();
                        void handleControlTick();

                private:
                        void receiveVoltageTarget(drivers::uart& uart);
                        void checkBusVoltage(uint16_t code);
                        void applyVoltageTarget(uint16_t bus_code);

                        volatile uint32_t target_mv = 0;
                        volatile bool target_pending = false;
                        volatile bool bus_undervoltage = true;
                        uint16_t measured_mv = 0;
                        uint16_t measured_capacitor_code = 0;
                        uint16_t commanded_duty_a = 0;
                        uint16_t commanded_duty_b = 0;
                        uint32_t windows = 0;
                        bool control_timeout = false;
                        uint32_t previous_tick = 0;
                        drivers::control_sample input{};
                        char stdout_buffer[256]{};
                        char command_line[32]{};
                        std::size_t command_length = 0;
                        bool command_overflow = false;
        };

        void run();
}
