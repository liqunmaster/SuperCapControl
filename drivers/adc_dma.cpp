/**
 * @brief TIM6 触发的双 ADC DMA 采样实现。
 * @file adc_dma.cpp
 */
#include "adc_dma.hpp"

#include "board.hpp"
#include "hrtim.hpp"

namespace
{
    bool prepareAdc(ADC_TypeDef* adc)
    {
        adc->CR = ADC_CR_ADVREGEN;
        board::delayUs(25);
        adc->CR |= ADC_CR_ADCAL;
        if (!board::waitBits(adc->CR, ADC_CR_ADCAL, 0))
        {
            return false;
        }
        board::delayUs(1);  ///< More than four ADC clock cycles after calibration.
        adc->ISR = ADC_ISR_ADRDY;
        adc->CR |= ADC_CR_ADEN;
        return board::waitBits(adc->ISR, ADC_ISR_ADRDY, ADC_ISR_ADRDY);
    }
    void configureDma(DMA_Channel_TypeDef* channel, volatile uint32_t* peripheral, volatile uint16_t* memory,
                      unsigned count)
    {
        channel->CCR = 0;
        channel->CPAR = reinterpret_cast<uint32_t>(peripheral);
        channel->CMAR = reinterpret_cast<uint32_t>(memory);
        channel->CNDTR = count;
        channel->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PL_1 | DMA_CCR_HTIE |
                       DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;
    }
}

namespace drivers
{
    /**
     * @brief 获取唯一采样驱动实例。
     * @return 驱动实例引用。
     */
    adc_dma& adc_dma::instance()
    {
        static adc_dma adc;
        return adc;
    }

    /**
     * @brief 校准 ADC1/2，并启动 1 kHz TIM6 触发及循环 DMA。
     * @pre 板级时钟和 DMA 时钟已初始化；仅在启动时调用一次。
     * @return 初始化成功返回 true，校准或就绪等待超时返回 false。
     */
    bool adc_dma::init()
    {
        for (unsigned pin = 5; pin <= 7; ++pin)
        {
            board::pinMode(GPIOA, pin, 3);
        }
        for (unsigned pin = 0; pin <= 2; ++pin)
        {
            board::pinMode(GPIOB, pin, 3);
        }
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
        (void)RCC->AHB2ENR;
        /// Synchronous HCLK /4 = 36 MHz, independent ADCs with the same TIM6 trigger.
        ADC12_COMMON->CCR = ADC_CCR_CKMODE_0 | ADC_CCR_CKMODE_1;
        if (!prepareAdc(ADC1) || !prepareAdc(ADC2))
        {
            stopOnError();
            return false;
        }
        constexpr uint32_t config = ADC_CFGR_DMAEN | ADC_CFGR_DMACFG | (13UL << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_EXTEN_0;
        ADC1->CFGR = config;
        ADC2->CFGR = config;
        /// 47.5-cycle acquisition for bring-up; validate settling time on the board.
        ADC1->SMPR2 = (4UL << 6) | (4UL << 15);  ///< IN12, IN15.
        ADC2->SMPR1 = (4UL << 9) | (4UL << 12);  ///< IN3, IN4.
        ADC2->SMPR2 = (4UL << 6) | (4UL << 9);   ///< IN12, IN13.
        ADC1->SQR1 = 1UL | (15UL << 6) | (12UL << 12);
        ADC2->SQR1 = 3UL | (13UL << 6) | (3UL << 12) | (4UL << 18) | (12UL << 24);
        DMAMUX1_Channel1->CCR = 5;   ///< ADC1 -> DMA1 channel 2.
        DMAMUX1_Channel2->CCR = 36;  ///< ADC2 -> DMA1 channel 3.
        DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;
        configureDma(DMA1_Channel2, &ADC1->DR, _adc1, 4);
        configureDma(DMA1_Channel3, &ADC2->DR, _adc2, 8);
        NVIC_SetPriority(DMA1_Channel2_IRQn, 3);
        NVIC_SetPriority(DMA1_Channel3_IRQn, 3);
        NVIC_EnableIRQ(DMA1_Channel2_IRQn);
        NVIC_EnableIRQ(DMA1_Channel3_IRQn);
        ADC1->IER = ADC_IER_OVRIE;
        ADC2->IER = ADC_IER_OVRIE;
        NVIC_SetPriority(ADC1_2_IRQn, 2);
        NVIC_EnableIRQ(ADC1_2_IRQn);
        TIM6->CR1 = 0;
        TIM6->PSC = board::CoreHz / 1000000 - 1;
        TIM6->ARR = 1000000 / board::SamplingHz - 1;
        TIM6->EGR = TIM_EGR_UG;
        TIM6->SR = 0;
        TIM6->CR2 = TIM_CR2_MMS_1;  ///< TRGO = update.
        ADC1->CR |= ADC_CR_ADSTART;
        ADC2->CR |= ADC_CR_ADSTART;
        TIM6->CR1 = TIM_CR1_CEN;
        return true;
    }
    void adc_dma::stopOnError()
    {
        TIM6->CR1 = 0;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        ADC1->IER = 0;
        ADC2->IER = 0;
        hrtim::disableOutputs();
        ++_errors;
    }

    /**
     * @brief 复制已完成的 DMA 半缓冲并处理积压或传输错误。
     * @param adc ADC 编号，仅允许 1 或 2，由对应 DMA 中断传入。
     */
    void adc_dma::handleDmaIrq(unsigned adc)
    {
        const unsigned shift = (adc == 1) ? 4 : 8;
        const uint32_t flags = (DMA1->ISR >> shift) & 15;
        DMA1->IFCR = 15UL << shift;
        if ((flags & 8) != 0 || (flags & 6) == 6)
        {
            stopOnError();
            return;
        }
        if (_errors != 0 || (flags & 6) == 0)
        {
            return;
        }
        const unsigned half = ((flags & 2) != 0) ? 1 : 0;
        __DMB();
        if (adc == 1)
        {
            for (unsigned i = 0; i < 2; ++i)
            {
                _latest1[i] = _adc1[half * 2 + i];
            }
            ++_sequence1;
        }
        else
        {
            for (unsigned i = 0; i < 4; ++i)
            {
                _latest2[i] = _adc2[half * 4 + i];
            }
            ++_sequence2;
        }
    }

    /**
     * @brief 处理 ADC overrun，停止采样并禁用功率输出。
     */
    void adc_dma::handleAdcIrq()
    {
        if (((ADC1->ISR | ADC2->ISR) & ADC_ISR_OVR) != 0)
        {
            stopOnError();
            ADC1->ISR = ADC_ISR_OVR;
            ADC2->ISR = ADC_ISR_OVR;
        }
    }

    /**
     * @brief 读取两组 ADC 序号匹配的最近一帧。
     * @param[out] sample 成功时写入原始采样值及序号。
     * @return 有有效匹配帧时返回 true，尚未完成或已发生错误时返回 false。
     */
    bool adc_dma::latest(adc_sample& sample) const
    {
        board::critical_section lock;
        if (_errors != 0 || _sequence1 == 0 || _sequence1 != _sequence2)
        {
            return false;
        }
        sample = {_latest2[0], _latest2[1], _latest2[2], _latest1[0], _latest1[1], _latest2[3], _sequence1};
        return true;
    }
}
/**
 * @brief 转发 ADC1 DMA 中断。
 */
extern "C" void DMA1_Channel2_IRQHandler()
{
    drivers::adc_dma::instance().handleDmaIrq(1);
}
/**
 * @brief 转发 ADC2 DMA 中断。
 */
extern "C" void DMA1_Channel3_IRQHandler()
{
    drivers::adc_dma::instance().handleDmaIrq(2);
}
/**
 * @brief 转发 ADC overrun 中断。
 */
extern "C" void ADC1_2_IRQHandler()
{
    drivers::adc_dma::instance().handleAdcIrq();
}
