/**
 * @file uart.cpp
 * @brief USART3 DMA transmit, circular receive and IDLE interrupt handling.
 */
#include "uart.hpp"

#include <algorithm>
#include <cstring>

#include "gpio.hpp"

namespace
{
        /**
         * @brief Configure a byte-wide USART DMA channel linked to a HAL handle.
         *
         */
        bool configureDma(DMA_HandleTypeDef& dma, DMA_Channel_TypeDef* instance, uint32_t request,
                              uint32_t direction, uint32_t mode)
        {
                dma.Instance = instance;
                dma.Init = {request, direction, DMA_PINC_DISABLE, DMA_MINC_ENABLE,
                                    DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE, mode, DMA_PRIORITY_LOW};
                return HAL_DMA_Init(&dma) == HAL_OK;
        }

        /**
         * @brief Change the three serialized UART IRQ enable states together.
         *
         */
        void setInterrupts(bool enabled)
        {
                if (enabled) {
                        HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
                        HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
                        HAL_NVIC_EnableIRQ(USART3_IRQn);
                }
                else {
                        HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
                        HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
                        HAL_NVIC_DisableIRQ(USART3_IRQn);
                }
        }
}

namespace drivers
{
        /**
         * @brief Return the sole owner of USART3 and DMA1 channels 5/6.
         *
         */
        uart& uart::instance()
        {
                static uart device;
                return device;
        }

        /**
         * @brief Start full duplex 115200 8N1 with circular receive DMA.
         *
         */
        bool uart::init()
        {
                __HAL_RCC_USART3_CLK_ENABLE();
                gpio::alternate(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_AF7_USART3, GPIO_PULLUP);
                _uart.Instance = USART3;
                _uart.Init = {board::UartBaud, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE,
                                      UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16,
                                      UART_ONE_BIT_SAMPLE_DISABLE,
                                      UART_PRESCALER_DIV1};
                if (!configureDma(_tx_dma, DMA1_Channel5, DMA_REQUEST_USART3_TX, DMA_MEMORY_TO_PERIPH, DMA_NORMAL) ||
                        !configureDma(_rx_dma, DMA1_Channel6, DMA_REQUEST_USART3_RX,
                                      DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR)) {
                        return false;
                }
                __HAL_LINKDMA(&_uart, hdmatx, _tx_dma);
                __HAL_LINKDMA(&_uart, hdmarx, _rx_dma);
                if (HAL_UART_Init(&_uart) != HAL_OK ||
                        HAL_UARTEx_ReceiveToIdle_DMA(&_uart, const_cast<uint8_t*>(_rx_dma_buffer),
                                                     RxDmaSize) != HAL_OK) {
                        return false;
                }

                HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, board::UartPriority, 0);
                HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, board::UartPriority, 0);
                HAL_NVIC_SetPriority(USART3_IRQn, board::UartPriority, 0);
                _ready = true;
                setInterrupts(true);
                return true;
        }

        /**
         * @brief Count queued TX bytes, including the active DMA segment.
         *
         */
        std::size_t uart::txPending() const
        {
                board::critical_section lock;
                return (_tx_head + TxSize - _tx_tail) % TxSize;
        }

        /**
         * @brief Queue as many bytes as fit and return immediately on backpressure.
         *
         */
        std::size_t uart::write(const char* data, std::size_t size)
        {
                if (!_ready || data == nullptr) {
                        return 0;
                }
                std::size_t written = 0;
                while (written < size) {
                        board::critical_section lock;
                        const unsigned room = (_tx_tail + TxSize - 1U - _tx_head) % TxSize;
                        const std::size_t count = std::min({size - written, static_cast<std::size_t>(room),
                                                                        static_cast<std::size_t>(TxSize - _tx_head),
                                                                        std::size_t{32}});
                        if (count == 0) {
                                break;
                        }
                        std::memcpy(&_tx[_tx_head], data + written, count);
                        _tx_head = (_tx_head + static_cast<unsigned>(count)) % TxSize;
                        written += count;
                        startTx();
                }
                return written;
        }

        /**
         * @brief Read one received byte without waiting.
         *
         */
        bool uart::read(uint8_t& byte)
        {
                board::critical_section lock;
                if (_rx_tail == _rx_head) {
                        return false;
                }
                byte = _rx[_rx_tail];
                _rx_tail = (_rx_tail + 1U) % RxQueueSize;
                return true;
        }

        /**
         * @brief Start DMA for the next contiguous transmit segment with UART IRQs serialized.
         *
         */
        void uart::startTx()
        {
                if (_recover || _tx_active != 0 || _tx_tail == _tx_head) {
                        return;
                }
                const unsigned count = _tx_head > _tx_tail ? _tx_head - _tx_tail : TxSize - _tx_tail;
                if (HAL_UART_Transmit_DMA(&_uart, &_tx[_tx_tail], static_cast<uint16_t>(count)) != HAL_OK) {
                        ++_tx_errors;
                        fault();
                        return;
                }
                _tx_active = count;
        }

        /**
         * @brief Release the transmitted segment after USART TC and start the next one.
         *
         */
        void uart::txComplete()
        {
                if (!_recover && _tx_active != 0) {
                        _tx_tail = (_tx_tail + _tx_active) % TxSize;
                        _tx_active = 0;
                        startTx();
                }
        }

        /**
         * @brief Return the current write position in the circular RX DMA buffer.
         *
         */
        unsigned uart::rxPosition() const
        {
                return (RxDmaSize - __HAL_DMA_GET_COUNTER(&_rx_dma)) % RxDmaSize;
        }

        /**
         * @brief Drain bytes once per DMA half, full or USART idle event.
         *
         */
        void uart::rxEvent()
        {
                if (_recover) {
                        return;
                }
                const uint32_t flags = DMA1->ISR;
                if ((flags & (DMA_ISR_HTIF6 | DMA_ISR_TCIF6)) == (DMA_ISR_HTIF6 | DMA_ISR_TCIF6)) {
                        DMA1->IFCR = DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6;
                        _rx_position = rxPosition();
                        ++_rx_errors;
                        return;
                }
                const unsigned position = rxPosition();
                __DMB();
                while (_rx_position != position) {
                        const unsigned next = (_rx_head + 1U) % RxQueueSize;
                        if (next == _rx_tail) {
                                ++_rx_errors;
                        }
                        else {
                                _rx[_rx_head] = _rx_dma_buffer[_rx_position];
                                _rx_head = next;
                        }
                        _rx_position = (_rx_position + 1U) % RxDmaSize;
                }
        }

        /**
         * @brief Quiesce hardware and discard the affected segment until task-side recovery.
         *
         */
        void uart::fault()
        {
                if (_recover) {
                        return;
                }
                _recover = true;
                setInterrupts(false);
                USART3->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
                ++_rx_errors;
                if (_tx_active != 0) {
                        ++_tx_errors;
                        _tx_tail = (_tx_tail + _tx_active) % TxSize;
                        _tx_active = 0;
                }
        }

        /**
         * @brief Retry a quiesced UART transfer from the owning task without masking the HAL timebase.
         *
         */
        void uart::service()
        {
                if (!_ready || !_recover) {
                        return;
                }
                if (_tx_dma.State == HAL_DMA_STATE_BUSY) {
                        HAL_DMA_Abort(&_tx_dma);
                }
                if (_rx_dma.State == HAL_DMA_STATE_BUSY) {
                        HAL_DMA_Abort(&_rx_dma);
                }
                if (HAL_UART_Abort(&_uart) != HAL_OK) {
                        return;
                }
                DMA1->IFCR = DMA_IFCR_CGIF5 | DMA_IFCR_CGIF6;
                HAL_NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
                HAL_NVIC_ClearPendingIRQ(DMA1_Channel6_IRQn);
                HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
                _rx_position = 0;
                if (HAL_UARTEx_ReceiveToIdle_DMA(&_uart, const_cast<uint8_t*>(_rx_dma_buffer), RxDmaSize) == HAL_OK) {
                        board::critical_section lock;
                        _recover = false;
                        setInterrupts(true);
                        startTx();
                }
        }

        /**
         * @brief Return cumulative transmit errors.
         *
         */
        uint32_t uart::txErrors() const
        {
                return _tx_errors;
        }

        /**
         * @brief Return cumulative receiver faults and discarded bytes.
         *
         */
        uint32_t uart::rxErrors() const
        {
                return _rx_errors;
        }

        /**
         * @brief Handle a UART DMA interrupt with HT/TC backlog detection on RX.
         *
         */
        void uart::handleDmaIrq(bool receive)
        {
                const uint32_t flags = DMA1->ISR;
                if ((flags & (receive ? DMA_ISR_TEIF6 : DMA_ISR_TEIF5)) != 0) {
                        fault();
                        return;
                }
                if (receive && (flags & (DMA_ISR_HTIF6 | DMA_ISR_TCIF6)) == (DMA_ISR_HTIF6 | DMA_ISR_TCIF6)) {
                        rxEvent();
                        return;
                }
                HAL_DMA_IRQHandler(receive ? &_rx_dma : &_tx_dma);
        }

        /**
         * @brief Dispatch USART IDLE, transmit complete and error events to HAL.
         *
         */
        void uart::handleUartIrq()
        {
                HAL_UART_IRQHandler(&_uart);
        }
}

/**
 * @brief Dispatch USART3 transmit DMA.
 *
 */
extern "C" void DMA1_Channel5_IRQHandler()
{
        drivers::uart::instance().handleDmaIrq(false);
}

/**
 * @brief Dispatch USART3 receive DMA.
 *
 */
extern "C" void DMA1_Channel6_IRQHandler()
{
        drivers::uart::instance().handleDmaIrq(true);
}

/**
 * @brief Dispatch USART3 line interrupts.
 *
 */
extern "C" void USART3_IRQHandler()
{
        drivers::uart::instance().handleUartIrq();
}

/**
 * @brief Publish USART transmit completion.
 *
 */
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*)
{
        drivers::uart::instance().txComplete();
}

/**
 * @brief Publish USART receive progress.
 *
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t)
{
        drivers::uart::instance().rxEvent();
}

/**
 * @brief Request task-side USART recovery after a line or DMA fault.
 *
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef*)
{
        drivers::uart::instance().fault();
}
