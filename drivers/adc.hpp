/** @file adc.hpp
 *  @brief Time-aligned dual ADC acquisition through circular DMA.
 */
#pragma once

#include "board.hpp"

namespace drivers
{
    /** @brief Six raw ADC readings belonging to one control window. */
    struct adc_sample
    {
        public:
            uint16_t capacitor_voltage;
            uint16_t capacitor_current;
            uint16_t chassis_voltage;
            uint16_t chassis_current;
            uint16_t output_voltage;
            uint16_t output_current;
            uint32_t sequence;  ///< control window number, wrapping modulo 2^32.
    };

    /** @brief ADC2 and the last ADC1 reading, plus rounded ADC1 channel means. */
    struct control_sample
    {
        public:
            adc_sample last;
            uint16_t mean_chassis_current;
            uint16_t mean_output_voltage;
    };

    /** @brief Common trigger, paired DMA half-buffers and latest-complete-block mailbox. */
    class adc
    {
        public:
            static adc& instance();
            bool init();
            bool readWindow(uint32_t sequence, control_sample& result);
            uint32_t errors() const;

            void handleDmaIrq(bool second);
            void handleAdcIrq();
            void fail();

        private:
            ADC_HandleTypeDef _adc1{};
            ADC_HandleTypeDef _adc2{};
            DMA_HandleTypeDef _dma1{};
            DMA_HandleTypeDef _dma2{};
            alignas(4) volatile uint16_t _raw1[2 * 4 * 2]{};
            alignas(4) volatile uint16_t _raw2[2 * 4]{};
            volatile uint32_t _errors = 0;
            volatile bool _running = false;
    };
}
