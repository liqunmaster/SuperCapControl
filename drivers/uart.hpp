/**
 * @file uart.hpp
 * @brief USART3 的 DMA 发送与循环 DMA 接收接口。
 */
#pragma once

#include <cstddef>

#include "board.hpp"

namespace drivers
{
        /**
         * @brief 使用定长缓冲区的非阻塞 USART3 驱动，前台负责读写。
         *
         */
        class uart
        {
                public:
                        static uart& instance();
                        bool init();
                        std::size_t txPending() const;
                        std::size_t write(const char* data, std::size_t size);
                        bool read(uint8_t& byte);
                        void service();
                        uint32_t txErrors() const;
                        uint32_t rxErrors() const;

                        void handleDmaIrq(bool receive);
                        void handleUartIrq();
                        void txComplete();
                        void rxEvent();
                        void fault();

                private:
                        static constexpr unsigned TxSize = 1024;
                        static constexpr unsigned RxDmaSize = 128;
                        static constexpr unsigned RxQueueSize = 256;

                        void startTx();
                        unsigned rxPosition() const;

                        UART_HandleTypeDef _uart{};
                        DMA_HandleTypeDef _tx_dma{};
                        DMA_HandleTypeDef _rx_dma{};
                        alignas(4) uint8_t _tx[TxSize]{};
                        alignas(4) volatile uint8_t _rx_dma_buffer[RxDmaSize]{};
                        uint8_t _rx[RxQueueSize]{};
                        unsigned _tx_head = 0;
                        unsigned _tx_tail = 0;
                        unsigned _tx_active = 0;
                        unsigned _rx_position = 0;
                        unsigned _rx_head = 0;
                        unsigned _rx_tail = 0;
                        volatile uint32_t _tx_errors = 0;
                        volatile uint32_t _rx_errors = 0;
                        volatile bool _recover = false;
                        bool _ready = false;
        };
}
