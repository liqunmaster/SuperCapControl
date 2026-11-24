/**
 * @file adc.hpp
 * @brief 通过循环 DMA 读取按 PWM 相位触发的双 ADC 采样。
 */
#pragma once

#include "board.hpp"

namespace drivers
{
        /**
         * @brief 同一控制窗口内六个通道的原始采样值。
         *
         */
        struct adc_sample
        {
                public:
                        uint16_t capacitor_voltage;
                        uint16_t capacitor_current;
                        uint16_t chassis_voltage;
                        uint16_t chassis_current;
                        uint16_t output_voltage;
                        uint16_t output_current;
                        /**
                         * @brief 控制窗口序号，按 32 位无符号整数自然回绕。
                         *
                         */
                        uint32_t sequence;
        };

        /**
         * @brief 窗口内最后一组采样值与四舍五入后的 ADC1 通道均值。
         *
         */
        struct control_sample
        {
                public:
                        adc_sample last;
                        uint16_t mean_chassis_current;
                        uint16_t mean_output_voltage;
        };

        /**
         * @brief 管理双 ADC 转换、成对的 DMA 半区和窗口读取。
         *
         */
        class adc
        {
                public:
                        static adc& instance();
                        bool init(void (*on_fault)());
                        bool readWindow(uint32_t sequence, control_sample& result);
                        uint32_t errors() const;

                        void handleDmaIrq(bool second);
                        void handleAdcIrq();
                        void fail();

                private:
                        void (*_on_fault)() = nullptr;
                        ADC_HandleTypeDef _adc1{};
                        ADC_HandleTypeDef _adc2{};
                        DMA_HandleTypeDef _dma1{};
                        DMA_HandleTypeDef _dma2{};
                        alignas(4) volatile uint16_t _raw1[2][board::adc_timing::scans_per_window][board::adc_timing::adc1_channels]{};
                        alignas(4) volatile uint16_t _raw2[2][board::adc_timing::scans_per_window][board::adc_timing::adc2_channels]{};
                        volatile uint32_t _errors = 0;
                        volatile bool _running = false;
        };
}
