/**
 * @file adc.cpp
 * @brief 由 HRTIM 触发双 ADC 扫描，使用循环 DMA 接收结果。
 */
#include "adc.hpp"

#include "gpio.hpp"

namespace
{
        /**
         * @brief 配置 ADC 规则转换序列和按半字传输的循环 DMA 通道。
         *
         */
        bool configureAdc(ADC_HandleTypeDef& adc, DMA_HandleTypeDef& dma, ADC_TypeDef* instance, DMA_Channel_TypeDef* channel, uint32_t request, const uint32_t* channels, unsigned count)
        {
                dma.Instance = channel;
                dma.Init     = {request, DMA_PERIPH_TO_MEMORY, DMA_PINC_DISABLE, DMA_MINC_ENABLE, DMA_PDATAALIGN_HALFWORD, DMA_MDATAALIGN_HALFWORD, DMA_CIRCULAR, DMA_PRIORITY_HIGH};
                if (HAL_DMA_Init(&dma) != HAL_OK) {
                        return false;
                }
                __HAL_LINKDMA(&adc, DMA_Handle, dma);

                adc.Instance                   = instance;
                adc.Init.ClockPrescaler        = board::adc_timing::prescaler;
                adc.Init.Resolution            = ADC_RESOLUTION_12B;
                adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
                adc.Init.GainCompensation      = 0;
                adc.Init.ScanConvMode          = ADC_SCAN_ENABLE;
                adc.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
                adc.Init.NbrOfConversion       = count;
                adc.Init.ExternalTrigConv      = instance == ADC1 ? ADC_EXTERNALTRIG_HRTIM_TRG6 : ADC_EXTERNALTRIG_HRTIM_TRG8;
                adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
                adc.Init.DMAContinuousRequests = ENABLE;
                adc.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
                if (HAL_ADC_Init(&adc) != HAL_OK) {
                        return false;
                }

                constexpr uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4};
                for (unsigned index = 0; index < count; ++index) {
                        ADC_ChannelConfTypeDef config{};
                        config.Channel      = channels[index];
                        config.Rank         = ranks[index];
                        config.SamplingTime = board::adc_timing::sample_time;
                        config.SingleDiff   = ADC_SINGLE_ENDED;
                        config.OffsetNumber = ADC_OFFSET_NONE;
                        if (HAL_ADC_ConfigChannel(&adc, &config) != HAL_OK) {
                                return false;
                        }
                }
                return HAL_ADCEx_Calibration_Start(&adc, ADC_SINGLE_ENDED) == HAL_OK;
        }
}

namespace drivers
{
        /**
         * @brief 获取管理 ADC1、ADC2 和 DMA1 通道 3、4 的驱动实例。
         *
         */
        adc& adc::instance()
        {
                static adc device;
                return device;
        }

        /**
         * @brief 启动 HRTIM 时基前，使能两路 ADC 及对应的 DMA。
         *
         */
        bool adc::init(void (*on_fault)())
        {
                if (on_fault == nullptr) {
                        return false;
                }
                _on_fault = on_fault;
                __HAL_RCC_ADC12_CLK_ENABLE();
                gpio::analog(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
                gpio::analog(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

                using timing = board::adc_timing;
                constexpr uint32_t adc1_channels[] = {ADC_CHANNEL_15, ADC_CHANNEL_12};
                constexpr uint32_t adc2_channels[] = {ADC_CHANNEL_4, ADC_CHANNEL_13, ADC_CHANNEL_3, ADC_CHANNEL_12};
                static_assert(sizeof(adc1_channels) / sizeof(adc1_channels[0]) == timing::adc1_channels, "ADC1 scan length mismatch");
                static_assert(sizeof(adc2_channels) / sizeof(adc2_channels[0]) == timing::adc2_channels, "ADC2 scan length mismatch");
                static_assert(adc2_channels[timing::capacitor_current_index] == ADC_CHANNEL_3, "Capacitor current must use PA6");
                if (!configureAdc(_adc1, _dma1, ADC1, DMA1_Channel3, DMA_REQUEST_ADC1, adc1_channels, timing::adc1_channels) ||
                        !configureAdc(_adc2, _dma2, ADC2, DMA1_Channel4, DMA_REQUEST_ADC2, adc2_channels, timing::adc2_channels) ||
                        HAL_ADC_Start_DMA(&_adc1, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(&_raw1[0][0][0])), sizeof(_raw1) / sizeof(uint16_t)) != HAL_OK ||
                        HAL_ADC_Start_DMA(&_adc2, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(&_raw2[0][0][0])), sizeof(_raw2) / sizeof(uint16_t)) != HAL_OK) {
                        fail();
                        return false;
                }

                HAL_NVIC_SetPriority(ADC1_2_IRQn, board::AdcErrorPriority, 0);
                HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, board::AdcDmaPriority, 0);
                HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, board::AdcDmaPriority, 0);
                HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
                HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
                HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
                _running = true;
                __HAL_DMA_DISABLE_IT(&_dma1, DMA_IT_HT | DMA_IT_TC);
                __HAL_DMA_DISABLE_IT(&_dma2, DMA_IT_HT | DMA_IT_TC);
                return true;
        }

        /**
         * @brief 调用 HAL 回调前检查 DMA 错误或积压的完成标志。
         *
         */
        void adc::handleDmaIrq(bool second)
        {
                const uint32_t shift = second ? 12U : 8U;
                const uint32_t flags = (DMA1->ISR >> shift) & 0xFU;
                if (!_running || (flags & 8U) != 0 || (flags & 6U) == 6U) {
                        DMA1->IFCR = 0xFU << shift;
                        if (_running) {
                                fail();
                        }
                        return;
                }
                HAL_DMA_IRQHandler(second ? &_dma2 : &_dma1);
        }

        /**
         * @brief 检测并锁存 ADC 溢出，阻止使用受影响的采样窗口。
         *
         */
        void adc::handleAdcIrq()
        {
                if (((ADC1->ISR | ADC2->ISR) & ADC_ISR_OVR) != 0) {
                        fail();
                        ADC1->ISR = ADC_ISR_OVR;
                        ADC2->ISR = ADC_ISR_OVR;
                }
        }

        /**
         * @brief 在控制节拍读取已完成的两个 DMA 半区，窗口序号从 1 开始。
         *
         */
        bool adc::readWindow(uint32_t sequence, control_sample& result)
        {
                if (!_running) {
                        return false;
                }
                const unsigned half     = (sequence - 1U) & 1U;
                const uint32_t expected = half == 0 ? DMA_ISR_HTIF3 | DMA_ISR_HTIF4 : DMA_ISR_TCIF3 | DMA_ISR_TCIF4;
                constexpr uint32_t flags = DMA_ISR_HTIF3 | DMA_ISR_HTIF4 | DMA_ISR_TCIF3 |
                                           DMA_ISR_TCIF4 | DMA_ISR_TEIF3 | DMA_ISR_TEIF4;
                if ((DMA1->ISR & flags) != expected || ((ADC1->ISR | ADC2->ISR) & ADC_ISR_OVR) != 0) {
                        fail();
                        return false;
                }
                DMA1->IFCR = expected;
                __DMB();
                using timing = board::adc_timing;
                const auto& adc1_scans = _raw1[half];
                const auto& last_adc1  = adc1_scans[timing::scans_per_window - 1U];
                const auto& last_adc2  = _raw2[half][timing::scans_per_window - 1U];
                uint32_t current = 0;
                uint32_t voltage = 0;
                for (const auto& scan : adc1_scans) {
                        current += scan[0];
                        voltage += scan[1];
                }
                result.last.capacitor_voltage = last_adc2[1];
                result.last.capacitor_current = last_adc2[timing::capacitor_current_index];
                result.last.chassis_voltage   = last_adc2[0];
                result.last.chassis_current   = last_adc1[0];
                result.last.output_voltage    = last_adc1[1];
                result.last.output_current    = last_adc2[3];
                result.last.sequence          = sequence;
                result.mean_chassis_current = static_cast<uint16_t>((current + timing::scans_per_window / 2U) / timing::scans_per_window);
                result.mean_output_voltage  = static_cast<uint16_t>((voltage + timing::scans_per_window / 2U) / timing::scans_per_window);
                return _running;
        }

        /**
         * @brief 停用两路采样 DMA，标记窗口无效并请求锁存关断。
         *
         */
        void adc::fail()
        {
                board::critical_section lock;
                if (_errors != 0) {
                        return;
                }
                _running = false;
                _errors = 1;
                ADC1->IER &= ~ADC_IER_OVRIE;
                ADC2->IER &= ~ADC_IER_OVRIE;
                DMA1_Channel3->CCR &= ~DMA_CCR_EN;
                DMA1_Channel4->CCR &= ~DMA_CCR_EN;
                if (_on_fault != nullptr) {
                        _on_fault();
                }
        }

        /**
         * @brief 采样正常时返回 0，故障锁存后返回 1。
         *
         */
        uint32_t adc::errors() const
        {
                return _errors;
        }
}

/**
 * @brief 分发 ADC1 的 DMA 中断。
 *
 */
extern "C" void DMA1_Channel3_IRQHandler()
{
        drivers::adc::instance().handleDmaIrq(false);
}

/**
 * @brief 分发 ADC2 的 DMA 中断。
 *
 */
extern "C" void DMA1_Channel4_IRQHandler()
{
        drivers::adc::instance().handleDmaIrq(true);
}

/**
 * @brief 处理 ADC1 或 ADC2 的硬件溢出中断。
 *
 */
extern "C" void ADC1_2_IRQHandler()
{
        drivers::adc::instance().handleAdcIrq();
}

/**
 * @brief 将 ADC 或 DMA 错误报告给采样驱动。
 *
 */
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*)
{
        drivers::adc::instance().fail();
}
