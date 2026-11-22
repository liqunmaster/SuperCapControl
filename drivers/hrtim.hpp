/**
 * @file hrtim.hpp
 * @brief HRTIM two-bridge bench PWM with paired DMA compare updates.
 */
#pragma once

#include "board.hpp"

namespace drivers
{
        /**
         * @brief HRTIM1 waveform owner. Power outputs require an explicit start.
         *
         */
        class hrtim
        {
                public:
                        static hrtim& instance();
                        bool init();
                        bool startTimebase();
                        static bool takeControlTick();
                        static bool controlTickPending();
                        bool start(uint16_t duty_a, uint16_t duty_b);
                        bool update(uint16_t duty_a, uint16_t duty_b);
                        void stop();
                        bool running() const;
                        bool faulted() const;
                        static void disableOutputs();

                        void handleDmaIrq(bool second);
                        void complete(bool second);

                private:
                        void cancelUpdate();
                        void stopHardware();

                        HRTIM_HandleTypeDef _timer{};
                        DMA_HandleTypeDef _dma_a{};
                        DMA_HandleTypeDef _dma_b{};
                        uint32_t _compare_a[2]{};
                        uint32_t _compare_b[2]{};
                        unsigned _pending = 0;
                        bool _await_latch = false;
                        bool _ready = false;
                        volatile bool _running = false;
                        volatile bool _fault = false;
        };
}
