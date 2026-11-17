/**
 * @brief UART DMA 与中断队列接口。
 * @file uart_dma.hpp
 */
#pragma once
#include <cstddef>
#include <cstdint>

/// @brief 外设驱动接口与实现。

namespace drivers
{
    /// Single foreground writer/reader; handlers never format strings or block.
    /**
     * @brief 前台单生产者 UART 日志发送与中断接收队列。
     * @note 禁止在中断上下文或多个并发生产者中调用 write。
     */
    class uart_dma
    {
        public:
            static uart_dma& instance();
            bool init();
            std::size_t write(const char* data, std::size_t size);
            bool read(uint8_t& byte);
            void handleDmaIrq();
            void handleUartIrq();
            /**
             * @brief 读取累计 TX DMA 错误次数。
             * @return 错误次数。
             */
            uint32_t txErrors() const
            {
                return _tx_errors;
            }
            /**
             * @brief 读取累计 RX 硬件错误及队列溢出次数。
             * @return 错误次数。
             */
            uint32_t rxErrors() const
            {
                return _rx_errors;
            }

        private:
            void startTx();
            alignas(4) uint8_t _tx[1024]{};
            uint8_t _rx[128]{};
            volatile unsigned _head = 0;
            volatile unsigned _tail = 0;
            volatile unsigned _active = 0;
            volatile unsigned _rx_head = 0;
            volatile unsigned _rx_tail = 0;
            volatile uint32_t _tx_errors = 0;
            volatile uint32_t _rx_errors = 0;
            bool _ready = false;
    };
}
