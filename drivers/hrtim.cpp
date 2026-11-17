#include "hrtim.hpp"

#include "board.hpp"

namespace
{
    alignas(4) uint32_t compare_values[2][64];
    volatile uint32_t dma_errors;
    bool initialized;
}

namespace drivers
{
    /**
     * @brief 禁用 HRTIM 输出，并将 PA8 至 PA11 的 GPIO 输出锁存值清零。
     * @pre GPIOA 时钟已开启；当前引脚保持 GPIO 输出模式。
     */
    void hrtim::disableOutputs()
    {
        if ((RCC->APB2ENR & RCC_APB2ENR_HRTIM1EN) != 0)
        {
            HRTIM1->sCommonRegs.ODISR = 0xFFF;
            __DSB();
        }
        GPIOA->BSRR = 0x0F00UL << 16;
    }

    /**
     * @brief 使能 HRTIM 时钟，禁用输出并初始化停止状态。
     */
    void hrtim::init()
    {
        RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;
        (void)RCC->APB2ENR;
        disableOutputs();
        HRTIM1->sMasterRegs.MCR = 0;
        for (unsigned timer = 0; timer < 2; ++timer)
        {
            HRTIM1->sTimerxRegs[timer].TIMxCR = HRTIM_TIMCR_PREEN;
            HRTIM1->sTimerxRegs[timer].TIMxDIER = 0;
        }
        initialized = true;
    }

    /**
     * @brief 复制比较值至驱动 SRAM 并配置 DMA，不启动传输。
     * @param timer 定时器索引：0 为 A，1 为 B。
     * @param values 比较值数组，每项范围为 3 至 0xFFFC。
     * @param count 比较值个数，范围为 1 至 64。
     * @return 参数有效、驱动已初始化且 A/B 计数器停止时返回 true。
     * @note 此处尚未校验比较值相对周期的约束，不可直接用于功率运行。
     */
    bool hrtim::prepareCompareDma(unsigned timer, const uint32_t* values, std::size_t count)
    {
        if (!initialized || timer > 1 || values == nullptr || count == 0 || count > 64)
        {
            return false;
        }
        if ((HRTIM1->sMasterRegs.MCR & (HRTIM_MCR_TACEN | HRTIM_MCR_TBCEN)) != 0)
        {
            return false;
        }
        for (std::size_t i = 0; i < count; ++i)
        {
            if (values[i] < 3 || values[i] > 0xFFFC)
            {
                return false;
            }
        }
        auto* dma = (timer == 0) ? DMA1_Channel4 : DMA1_Channel5;
        auto* mux = (timer == 0) ? DMAMUX1_Channel3 : DMAMUX1_Channel4;
        dma->CCR = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            compare_values[timer][i] = values[i];
        }
        mux->CCR = 96 + timer;
        dma->CPAR = reinterpret_cast<uint32_t>(&HRTIM1->sTimerxRegs[timer].CMP1xR);
        dma->CMAR = reinterpret_cast<uint32_t>(compare_values[timer]);
        dma->CNDTR = static_cast<uint32_t>(count);
        DMA1->IFCR = 15UL << ((timer + 3) * 4);
        dma->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PL | DMA_CCR_TEIE |
                   DMA_CCR_TCIE;  // DMA 通道保持禁用。
        const auto irq = (timer == 0) ? DMA1_Channel4_IRQn : DMA1_Channel5_IRQn;
        NVIC_SetPriority(irq, 1);
        NVIC_EnableIRQ(irq);
        return true;
    }

    /**
     * @brief 清除指定 HRTIM DMA 通道标志，错误时关闭输出。
     * @param timer 定时器索引，仅允许 0 或 1，由对应 DMA 中断传入。
     */
    void hrtim::handleDmaIrq(unsigned timer)
    {
        const unsigned shift = (timer + 3) * 4;
        const uint32_t flags = (DMA1->ISR >> shift) & 15;
        DMA1->IFCR = 15UL << shift;
        if ((flags & 8) != 0)
        {
            ++dma_errors;
            disableOutputs();
        }
    }

    /**
     * @brief 读取 HRTIM DMA 累计错误次数。
     * @return 错误次数。
     */
    uint32_t hrtim::errors()
    {
        return dma_errors;
    }
}

extern "C" void DMA1_Channel4_IRQHandler()
{
    drivers::hrtim::handleDmaIrq(0);
}

extern "C" void DMA1_Channel5_IRQHandler()
{
    drivers::hrtim::handleDmaIrq(1);
}
