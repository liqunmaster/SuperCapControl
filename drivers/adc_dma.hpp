/**
 * @brief 双 ADC DMA 采样接口。
 * @file adc_dma.hpp
 */
#pragma once
#include <cstdint>

/// @brief 外设驱动接口与实现。

namespace drivers
{
    /**
     * @brief 同一触发序号对应的六路 12 位 ADC 原始码。
     */
    struct adc_sample
    {
        public:
            uint16_t capacitor_voltage;  ///< PA5 电容电压原始码。
            uint16_t capacitor_current;  ///< PA6 电容电流原始码。
            uint16_t chassis_voltage;    ///< PA7 底盘电压原始码。
            uint16_t chassis_current;    ///< PB0 底盘电流原始码。
            uint16_t output_voltage;     ///< PB1 输出电压原始码。
            uint16_t output_current;     ///< PB2 输出电流原始码。
            uint32_t sequence;           ///< 两组 ADC 匹配的触发序号。
    };

    /**
     * @brief TIM6 硬件触发的双 ADC DMA 采样驱动。
     */
    class adc_dma
    {
        public:
            static adc_dma& instance();
            bool init();
            bool latest(adc_sample& sample) const;
            void handleDmaIrq(unsigned adc);
            void handleAdcIrq();
            /**
             * @brief 读取采样错误次数。
             * @return 累计错误次数，非零表示采样已停止。
             */
            uint32_t errors() const
            {
                return _errors;
            }

        private:
            void stopOnError();
            alignas(4) volatile uint16_t _adc1[4]{};
            alignas(4) volatile uint16_t _adc2[8]{};
            volatile uint16_t _latest1[2]{};
            volatile uint16_t _latest2[4]{};
            volatile uint32_t _sequence1 = 0;
            volatile uint32_t _sequence2 = 0;
            volatile uint32_t _errors = 0;
    };
}
