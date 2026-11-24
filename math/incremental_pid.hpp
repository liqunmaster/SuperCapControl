/**
 * @file incremental_pid.hpp
 * @brief 目标与测量分别加权的离散增量控制器。
 */
#pragma once

namespace math
{
        struct pid_config
        {
                float kp_target;
                float kp_measurement;
                float ki_step;
                float kd_step;
        };

        class incremental_pid
        {
                public:
                        explicit incremental_pid(pid_config gains);
                        void seed(float target, float measured);
                        float update(float target, float measured);

                private:
                        pid_config gains;
                        float previous_target = 0.0f;
                        float previous_measurement = 0.0f;
                        float older_measurement = 0.0f;
        };
}
