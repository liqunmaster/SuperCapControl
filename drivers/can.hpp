/**
 * @brief 经典 CAN 数据帧和驱动接口。
 * @file can.hpp
 */
#pragma once
#include <cstdint>

/// @brief 外设驱动接口与实现。

namespace drivers
{
    /**
     * @brief 经典 CAN 标准或扩展数据帧，最多 8 字节。
     */
    struct can_frame
    {
        public:
            uint32_t id = 0;        ///< 标准 11 位或扩展 29 位标识符。
            bool extended = false;  ///< 为 true 时使用扩展标识符。
            uint8_t size = 0;       ///< 有效数据长度，范围为 0 至 8。
            uint8_t data[8]{};      ///< 帧数据缓冲。
    };
    /**
     * @brief FDCAN3 经典 CAN 1 Mbps 收发与中断接收队列。
     */
    class can
    {
        public:
            static can& instance();
            bool init();
            bool send(const can_frame& frame);
            bool receive(can_frame& frame);

            void handleIrq();

            /// @brief 返回接收丢帧、非法帧和 bus-off 的累计错误次数。
            uint32_t errors() const
            {
                return _errors;
            }

            /// @brief 返回锁存的 bus-off 状态，发生 bus-off 后持续返回 true。
            bool busOff() const
            {
                return _bus_off;
            }

        private:
            can_frame _rx[16]{};
            volatile unsigned _head = 0;
            volatile unsigned _tail = 0;
            volatile uint32_t _errors = 0;
            volatile bool _bus_off = false;
            bool _ready = false;
    };
}
