/**
 * @file control.hpp
 * @brief 裸机控制应用及其运行状态。
 */
#pragma once

#include <cstddef>

#include "adc.hpp"
#include "board.hpp"
#include "voltage.hpp"

namespace drivers
{
        class uart;
}

namespace application
{
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
                        static constexpr uint16_t adc_full_scale                = 4095U;
                        static constexpr uint16_t adc_reference_mv              = 3000U;
                        static constexpr unsigned voltage_divider_gain          = (100U + 10U) / 10U;
                        static constexpr unsigned voltage_full_scale_mv        = voltage_divider_gain * adc_reference_mv;
                        static constexpr float voltage_mv_per_code = static_cast<float>(voltage_full_scale_mv) / adc_full_scale;

                        void checkBusVoltage(uint16_t code);

                        voltage_loop regulator;
                        volatile uint32_t target_mv = 0;
                        bool bus_undervoltage = true;
                        uint16_t measured_mv = 0;
                        uint16_t measured_capacitor_code = 0;
                        uint32_t windows = 0;
                        bool control_timeout = false;
                        uint32_t previous_tick = 0;
                        char stdout_buffer[256]{};
        };

        void run();
}
