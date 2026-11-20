/**
 * @file can.hpp
 * @brief Interrupt-driven classic CAN using the STM32 FDCAN HAL.
 */
#pragma once

#include "board.hpp"

namespace drivers
{
        /**
         * @brief Classic CAN data frame and local receive timestamp.
         *
         */
        struct can_frame
        {
                public:
                        uint32_t id = 0;
                        bool extended = false;
                        uint8_t size = 0;
                        uint8_t data[8]{};
                        uint32_t received_ms = 0;
        };

        /**
         * @brief FDCAN3 transmit FIFO and bounded software receive queue.
         *
         */
        class can
        {
                public:
                        static can& instance();
                        bool init(void (*notify)() = nullptr);
                        bool send(const can_frame& frame);
                        bool receive(can_frame& frame);
                        uint32_t errors() const;
                        bool busOff() const;
                        void handleIrq();

                private:
                        FDCAN_HandleTypeDef _can{};
                        void (*_notify)() = nullptr;
                        can_frame _rx[16]{};
                        volatile unsigned _head = 0;
                        volatile unsigned _tail = 0;
                        volatile uint32_t _errors = 0;
                        volatile bool _bus_off = false;
                        bool _ready = false;
        };
}
