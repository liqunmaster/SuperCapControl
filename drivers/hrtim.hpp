/**
 * @file hrtim.hpp
 * @brief HRTIM 双桥 PWM 接口，支持成对更新比较值。
 */
#pragma once

#include "board.hpp"

namespace drivers
{
        /**
         * @brief 管理 HRTIM1 波形，显式启动后才使能功率输出。
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


                private:
                        void releaseCompareUpdate();
                        void cancelUpdate();
                        void stopHardware();

                        HRTIM_HandleTypeDef _timer{};
                        uint32_t _compare_a[2]{};
                        uint32_t _compare_b[2]{};
                        bool _await_latch = false;
                        bool _ready = false;
                        volatile bool _running = false;
                        volatile bool _fault = false;
        };
}
