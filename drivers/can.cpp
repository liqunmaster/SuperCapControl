/**
 * @file can.cpp
 * @brief HAL FDCAN3 classic CAN with interrupt reception.
 */
#include "can.hpp"

#include <cstring>

#include "gpio.hpp"

namespace
{
        constexpr uint32_t receive_events = FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
        constexpr uint32_t events         = receive_events | FDCAN_IT_BUS_OFF;
}

namespace drivers
{
        /**
         * @brief Return the sole owner of FDCAN3.
         *
         */
        can& can::instance()
        {
                static can device;
                return device;
        }

        /**
         * @brief Start classic CAN at 1 Mbps after confirming the transceiver pinout.
         * @param notify Optional nonblocking ISR callback for a received frame or status change.
         *
         */
        bool can::init(void (*notify)())
        {
                RCC_PeriphCLKInitTypeDef clock{};
                clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
                clock.FdcanClockSelection  = RCC_FDCANCLKSOURCE_HSE;
                if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
                        return false;
                }
                __HAL_RCC_FDCAN_CLK_ENABLE();
                gpio::alternate(GPIOB, GPIO_PIN_3 | GPIO_PIN_4, GPIO_AF11_FDCAN3);

                _can.Instance                  = FDCAN3;
                _can.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
                _can.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
                _can.Init.Mode                 = FDCAN_MODE_NORMAL;
                _can.Init.AutoRetransmission   = ENABLE;
                _can.Init.NominalPrescaler     = 1;
                _can.Init.NominalSyncJumpWidth = 4;
                _can.Init.NominalTimeSeg1      = 18;
                _can.Init.NominalTimeSeg2      = 5;
                _can.Init.DataPrescaler        = 1;
                _can.Init.DataSyncJumpWidth    = 1;
                _can.Init.DataTimeSeg1         = 1;
                _can.Init.DataTimeSeg2         = 1;
                _can.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;
                if (HAL_FDCAN_Init(&_can) != HAL_OK ||
                        HAL_FDCAN_ConfigGlobalFilter(&_can, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
                        HAL_FDCAN_ConfigInterruptLines(&_can, events, FDCAN_INTERRUPT_LINE0) != HAL_OK ||
                        HAL_FDCAN_ActivateNotification(&_can, events, 0) != HAL_OK ||
                        HAL_FDCAN_Start(&_can) != HAL_OK) {
                        ++_errors;
                        return false;
                }

                _notify = notify;
                _ready = true;
                HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, board::CanPriority, 0);
                HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
                return true;
        }

        /**
         * @brief Queue a valid classic data frame without blocking in the caller task.
         *
         */
        bool can::send(const can_frame& frame)
        {
                if (!_ready || _bus_off || frame.size > 8 || frame.id > (frame.extended ? 0x1FFFFFFFU : 0x7FFU)) {
                        return false;
                }
                FDCAN_TxHeaderTypeDef header{};
                header.Identifier          = frame.id;
                header.IdType              = frame.extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
                header.TxFrameType         = FDCAN_DATA_FRAME;
                header.DataLength          = frame.size;
                header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
                header.BitRateSwitch       = FDCAN_BRS_OFF;
                header.FDFormat            = FDCAN_CLASSIC_CAN;
                header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

                board::critical_section lock;
                return !_bus_off && HAL_FDCAN_GetTxFifoFreeLevel(&_can) != 0 && HAL_FDCAN_AddMessageToTxFifoQ(&_can, &header, frame.data) == HAL_OK;
        }

        /**
         * @brief Copy and remove the oldest queued frame.
         *
         */
        bool can::receive(can_frame& frame)
        {
                board::critical_section lock;
                if (_tail == _head) {
                        return false;
                }
                frame = _rx[_tail];
                _tail = (_tail + 1U) % 16U;
                return true;
        }

        /**
         * @brief Count hardware loss, software overflow, invalid frames and bus-off.
         *
         */
        uint32_t can::errors() const
        {
                return _errors;
        }

        /**
         * @brief Return the latched bus-off state.
         *
         */
        bool can::busOff() const
        {
                return _bus_off;
        }

        /**
         * @brief Acknowledge HAL events and drain at most the hardware FIFO depth.
         *
         */
        void can::handleIrq()
        {
                const uint32_t errors_before = _errors;
                const unsigned head_before   = _head;
                if ((FDCAN3->IR & FDCAN_IR_RF0L) != 0) {
                        ++_errors;
                }
                if (!_bus_off && (FDCAN3->PSR & FDCAN_PSR_BO) != 0) {
                        _bus_off = true;
                        ++_errors;
                }
                HAL_FDCAN_IRQHandler(&_can);

                for (unsigned index = 0; index < 3 && HAL_FDCAN_GetRxFifoFillLevel(&_can, FDCAN_RX_FIFO0) != 0; ++index) {
                        FDCAN_RxHeaderTypeDef header{};
                        uint8_t payload[64];  ///< HAL can copy an FD-sized DLC before the driver validates it.
                        if (HAL_FDCAN_GetRxMessage(&_can, FDCAN_RX_FIFO0, &header, payload) != HAL_OK) {
                                ++_errors;
                                break;
                        }
                        const unsigned next = (_head + 1U) % 16U;
                        if (next == _tail || header.DataLength > 8 || header.FDFormat != FDCAN_CLASSIC_CAN || header.RxFrameType != FDCAN_DATA_FRAME) {
                                ++_errors;
                                continue;
                        }
                        auto& frame = _rx[_head];
                        frame.id          = header.Identifier;
                        frame.extended    = header.IdType == FDCAN_EXTENDED_ID;
                        frame.size        = static_cast<uint8_t>(header.DataLength);
                        frame.received_ms = HAL_GetTick();
                        std::memcpy(frame.data, payload, frame.size);
                        __DMB();
                        _head = next;
                }
                if (HAL_FDCAN_GetRxFifoFillLevel(&_can, FDCAN_RX_FIFO0) != 0) {
                        HAL_NVIC_SetPendingIRQ(FDCAN3_IT0_IRQn);
                }
                if (_notify != nullptr && (head_before != _head || errors_before != _errors)) {
                        _notify();
                }
        }
}

/**
 * @brief Route FDCAN3 interrupt line zero to its owner.
 *
 */
extern "C" void FDCAN3_IT0_IRQHandler()
{
        drivers::can::instance().handleIrq();
}
