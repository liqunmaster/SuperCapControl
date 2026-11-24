/**
 * @file incremental_pid.cpp
 * @brief 按目标差分、测量差分和当前误差计算单拍增量。
 */
#include "incremental_pid.hpp"

namespace math
{
        /**
         * @brief 保存当前采样周期对应的离散增益。
         *
         */
        incremental_pid::incremental_pid(pid_config config) : gains(config)
        {
        }

        /**
         * @brief 用同一实测值填充两拍历史，切入闭环时抑制虚假差分。
         *
         */
        void incremental_pid::seed(float target, float measured)
        {
                previous_target      = target;
                previous_measurement = measured;
                older_measurement    = measured;
        }

        /**
         * @brief 计算本拍输出增量，由调用者累加并限制最终命令。
         *
         */
        float incremental_pid::update(float target, float measured)
        {
                const float target_step = target - previous_target;
                const float measured_step = measured - previous_measurement;
                const float curvature = measured_step - (previous_measurement - older_measurement);
                const float delta = gains.kp_target * target_step - gains.kp_measurement * measured_step + gains.ki_step * (target - measured) + gains.kd_step * curvature;
                older_measurement    = previous_measurement;
                previous_measurement = measured;
                previous_target      = target;
                return delta;
        }
}
