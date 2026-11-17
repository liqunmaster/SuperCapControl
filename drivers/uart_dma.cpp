/**
 * @brief UART DMA 队列和中断处理实现。
 * @file uart_dma.cpp
 */
#include "uart_dma.hpp"

#include <cstring>

#include "board.hpp"

namespace drivers
{
    /**
     * @brief 获取唯一串口驱动实例。
     * @return 驱动实例引用。
     */
    uart_dma& uart_dma::instance()
    {
        static uart_dma uart;
        return uart;
    }

    /**
     * @brief 初始化 USART3 为 115200 8N1，并配置 TX DMA 和 RX 中断。
     * @pre 板级时钟和 DMA 时钟已初始化；仅在启动时调用一次。
     * @return 收发器就绪时返回 true，等待超时返回 false。
     */
    bool uart_dma::init()
    {
        board::pinMode(GPIOB, 10, 2, 7);
        board::pinMode(GPIOB, 11, 2, 7);
        GPIOB->PUPDR |= 1UL << 22;
        RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
        (void)RCC->APB1ENR1;
        USART3->CR1 = 0;
        USART3->CR2 = 0;  ///< One stop bit.
        USART3->BRR = (board::CoreHz + board::UartBaud / 2) / board::UartBaud;
        USART3->CR3 = USART_CR3_DMAT | USART_CR3_EIE;
        DMA1_Channel1->CCR = 0;
        DMAMUX1_Channel0->CCR = 29;  ///< RM0440: USART3_TX.
        DMA1_Channel1->CPAR = reinterpret_cast<uint32_t>(&USART3->TDR);
        DMA1->IFCR = DMA_IFCR_CGIF1;
        NVIC_SetPriority(DMA1_Channel1_IRQn, 10);
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        NVIC_SetPriority(USART3_IRQn, 11);
        NVIC_EnableIRQ(USART3_IRQn);
        USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_PEIE;
        _ready = board::waitBits(USART3->ISR, USART_ISR_TEACK | USART_ISR_REACK, USART_ISR_TEACK | USART_ISR_REACK);
        return _ready;
    }
    void uart_dma::startTx()
    {
        if (_active != 0 || _tail == _head)
        {
            return;
        }
        _active = (_head > _tail) ? _head - _tail : sizeof(_tx) - _tail;
        DMA1_Channel1->CCR = 0;
        DMA1->IFCR = DMA_IFCR_CGIF1;
        DMA1_Channel1->CMAR = reinterpret_cast<uint32_t>(&_tx[_tail]);
        DMA1_Channel1->CNDTR = _active;
        __DMB();
        DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;
    }

    /**
     * @brief 将日志复制至有界 TX 队列并启动 DMA，不等待发送完成。
     * @param data 输入字节缓冲。
     * @param size 请求写入的字节数。
     * @return 实际入队字节数；队列满时可短写，未就绪或中断上下文返回 0。
     */
    std::size_t uart_dma::write(const char* data, std::size_t size)
    {
        if (!_ready || data == nullptr || __get_IPSR() != 0)
        {
            return 0;
        }
        std::size_t written = 0;
        while (written < size)
        {
            /// Copy <= 64 bytes in each critical section, bounded interrupt latency.
            board::critical_section lock;
            const unsigned free = (_tail + sizeof(_tx) - _head - 1) % sizeof(_tx);
            if (free == 0)
            {
                break;
            }
            unsigned count = static_cast<unsigned>((size - written > 64) ? 64 : size - written);
            if (count > free)
            {
                count = free;
            }
            if (count > sizeof(_tx) - _head)
            {
                count = sizeof(_tx) - _head;
            }
            std::memcpy(&_tx[_head], data + written, count);
            _head = (_head + count) % sizeof(_tx);
            written += count;
            startTx();
        }
        return written;
    }

    /**
     * @brief 处理 TX DMA 完成与错误标志并推进发送队列。
     */
    void uart_dma::handleDmaIrq()
    {
        const uint32_t status = DMA1->ISR;
        DMA1->IFCR = DMA_IFCR_CGIF1;
        if ((status & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1)) == 0)
        {
            return;
        }
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        if ((status & DMA_ISR_TEIF1) != 0)
        {
            ++_tx_errors;
        }
        /// Drop the active chunk on error, do not replay potentially transmitted bytes.
        _tail = (_tail + _active) % sizeof(_tx);
        _active = 0;
        startTx();
    }

    /**
     * @brief 处理 RX 字节及串口错误，队列满时丢弃新字节。
     */
    void uart_dma::handleUartIrq()
    {
        const uint32_t status = USART3->ISR;
        if ((status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0)
        {
            USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
            ++_rx_errors;
        }
        if ((status & USART_ISR_RXNE) != 0)
        {
            const auto byte = static_cast<uint8_t>(USART3->RDR);
            const unsigned next = (_rx_head + 1) % sizeof(_rx);
            if (next == _rx_tail)
            {
                ++_rx_errors;
            }
            else
            {
                _rx[_rx_head] = byte;
                __DMB();
                _rx_head = next;
            }
        }
    }

    /**
     * @brief 从 RX 队列取出一个字节。
     * @param[out] byte 成功时写入接收字节。
     * @return 队列非空时返回 true，否则返回 false。
     */
    bool uart_dma::read(uint8_t& byte)
    {
        board::critical_section lock;
        if (_rx_tail == _rx_head)
        {
            return false;
        }
        byte = _rx[_rx_tail];
        _rx_tail = (_rx_tail + 1) % sizeof(_rx);
        return true;
    }
}
/**
 * @brief 转发 UART TX DMA 中断。
 */
extern "C" void DMA1_Channel1_IRQHandler()
{
    drivers::uart_dma::instance().handleDmaIrq();
}
/**
 * @brief 转发 UART RX 和错误中断。
 */
extern "C" void USART3_IRQHandler()
{
    drivers::uart_dma::instance().handleUartIrq();
}
