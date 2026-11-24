#pragma once

#include "board.hpp"
#include "incremental_pid.hpp"

namespace application
{
        /**
         * @brief 保存已生效的电压环状态，成对提交两侧 PWM 命令。
         *
         */
        class voltage_loop
        {
                public:
                        static constexpr uint32_t voltage_update_divider = 32U;
                        void apply(uint32_t requested, float bus, float measured, bool bench_mode);
                        void stop();

                private:
                        // 依次为目标比例、测量比例、每拍积分和测量二阶差分增益。
                        math::incremental_pid voltage_pid{{0.1f, 0.2f, 0.001f, 0.0f}};
                        float accepted_ratio = 0.0f;
                        bool loop_active = false;
                        uint16_t commanded_duty_a = 0;
                        uint16_t commanded_duty_b = 0;
        };
}
