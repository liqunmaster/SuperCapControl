/**
 * @file adc.cpp
 * @brief TIM6-triggered dual ADC with HAL ADC/DMA and a complete-block mailbox.
 */
#include "adc.hpp"

#include "gpio.hpp"

namespace
{
        constexpr uint32_t adc1_channels[] = {ADC_CHANNEL_15, ADC_CHANNEL_12};
        constexpr uint32_t adc2_channels[] = {ADC_CHANNEL_3, ADC_CHANNEL_13, ADC_CHANNEL_4, ADC_CHANNEL_12};
        constexpr uint32_t ranks[] = {ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4};
        /**
         * @brief Configure a regular sequence and the circular, half-word DMA channel it owns.
         *
         */
        bool configureAdc(ADC_HandleTypeDef& adc, DMA_HandleTypeDef& dma, ADC_TypeDef* instance,
                              DMA_Channel_TypeDef* channel, uint32_t request, const uint32_t* channels, unsigned count)
        {
                dma.Instance = channel;
                dma.Init = {request, DMA_PERIPH_TO_MEMORY, DMA_PINC_DISABLE, DMA_MINC_ENABLE,
                                    DMA_PDATAALIGN_HALFWORD, DMA_MDATAALIGN_HALFWORD, DMA_CIRCULAR, DMA_PRIORITY_HIGH};
                if (HAL_DMA_Init(&dma) != HAL_OK) {
                        return false;
                }
                __HAL_LINKDMA(&adc, DMA_Handle, dma);

                adc.Instance = instance;
                adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
                adc.Init.Resolution = ADC_RESOLUTION_12B;
                adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
                adc.Init.GainCompensation = 0;
                adc.Init.ScanConvMode = ADC_SCAN_ENABLE;
                adc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
                adc.Init.NbrOfConversion = count;
                adc.Init.ExternalTrigConv = instance == ADC1 ?
                        ADC_EXTERNALTRIG_HRTIM_TRG1 : ADC_EXTERNALTRIG_HRTIM_TRG3;
                adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
                adc.Init.DMAContinuousRequests = ENABLE;
                adc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
                if (HAL_ADC_Init(&adc) != HAL_OK) {
                        return false;
                }

                for (unsigned index = 0; index < count; ++index) {
                        ADC_ChannelConfTypeDef config{};
                        config.Channel = channels[index];
                        config.Rank = ranks[index];
                        config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
                        config.SingleDiff = ADC_SINGLE_ENDED;
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
         * @brief Return the sole owner of ADC1/2 and DMA1 channels 3/4.
         *
         */
        adc& adc::instance()
        {
                static adc device;
                return device;
        }

        /**
         * @brief Arm both ADC DMA channels before starting their common hardware trigger.
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



                if (!configureAdc(_adc1, _dma1, ADC1, DMA1_Channel3, DMA_REQUEST_ADC1, adc1_channels, 2) ||
                        !configureAdc(_adc2, _dma2, ADC2, DMA1_Channel4, DMA_REQUEST_ADC2, adc2_channels, 4) ||
                        HAL_ADC_Start_DMA(&_adc1, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(_raw1)),
                                                      sizeof(_raw1) / sizeof(_raw1[0])) != HAL_OK ||
                        HAL_ADC_Start_DMA(&_adc2, reinterpret_cast<uint32_t*>(const_cast<uint16_t*>(_raw2)),
                                                      sizeof(_raw2) / sizeof(_raw2[0])) != HAL_OK) {
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
         * @brief Reject DMA flag backlog before dispatching the HAL DMA callback.
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
         * @brief Latch an ADC overrun before any affected DMA block can be published.
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
         * @brief Consume the completed DMA halves at the hardware deadline. Sequence starts at one.
         *
         */
        bool adc::readWindow(uint32_t sequence, control_sample& result)
        {
                if (!_running) {
                        return false;
                }
                const unsigned half = (sequence - 1U) & 1U;
                const uint32_t expected = half == 0 ? DMA_ISR_HTIF3 | DMA_ISR_HTIF4 : DMA_ISR_TCIF3 | DMA_ISR_TCIF4;
                constexpr uint32_t flags = DMA_ISR_HTIF3 | DMA_ISR_HTIF4 | DMA_ISR_TCIF3 | DMA_ISR_TCIF4 |
                                                   DMA_ISR_TEIF3 | DMA_ISR_TEIF4;
                if ((DMA1->ISR & flags) != expected || ((ADC1->ISR | ADC2->ISR) & ADC_ISR_OVR) != 0) {
                        fail();
                        return false;
                }
                DMA1->IFCR = expected;
                __DMB();
                const volatile uint16_t* first = &_raw1[8 * half];
                const volatile uint16_t* second = &_raw2[4 * half];
                uint32_t current = 0;
                uint32_t voltage = 0;
                for (unsigned i = 0; i < 4; ++i) {
                        current += first[2 * i];
                        voltage += first[2 * i + 1];
                }
                result.last = {second[1], second[0], second[2], first[6], first[7], second[3], sequence};
                result.mean_chassis_current = static_cast<uint16_t>((current + 2) / 4);
                result.mean_output_voltage = static_cast<uint16_t>((voltage + 2) / 4);
                return _running;
        }

        /**
         * @brief Stop triggering and DMA, invalidate the mailbox, and latch PWM off.
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
         * @brief Return zero while healthy and one after the acquisition fault has latched.
         *
         */
        uint32_t adc::errors() const
        {
                return _errors;
        }
}

/**
 * @brief Dispatch the ADC1 DMA channel.
 *
 */
extern "C" void DMA1_Channel3_IRQHandler()
{
        drivers::adc::instance().handleDmaIrq(false);
}

/**
 * @brief Dispatch the ADC2 DMA channel.
 *
 */
extern "C" void DMA1_Channel4_IRQHandler()
{
        drivers::adc::instance().handleDmaIrq(true);
}

/**
 * @brief Dispatch hardware overrun for either ADC.
 *
 */
extern "C" void ADC1_2_IRQHandler()
{
        drivers::adc::instance().handleAdcIrq();
}

/**
 * @brief Report an ADC or DMA error to the acquisition driver.
 *
 */
extern "C" void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*)
{
        drivers::adc::instance().fail();
}
