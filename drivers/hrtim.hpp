/**
 * @brief HRTIM 输出保护和 DMA 准备接口。
 * @file hrtim.hpp
 */
#pragma once
#include <cstddef>
#include <cstdint>

/// @brief 外设驱动接口与实现。

namespace drivers
{
    /// Bring-up guard and compare DMA preparation. No output-enable API until the
    /// power-stage timing, dead time and hardware fault path have been specified.
    /**
     * @brief HRTIM 输出关闭保护及比较值 DMA 准备接口。
     * @note 当前不启用计数器、DMA 请求或功率输出。
     */
    class hrtim
    {
        public:
            static void init();
            static void disableOutputs();
            static bool prepareCompareDma(unsigned timer, const uint32_t* values, std::size_t count);
            static void handleDmaIrq(unsigned timer);
            static uint32_t errors();
    };
}
