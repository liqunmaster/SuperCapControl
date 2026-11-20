/**
 * @file incremental_pid.hpp
 * @brief Bounded incremental PID interface and configuration.
 */
#pragma once

namespace math
{
        /**
         * @brief Gains, sampling interval in seconds, and output bounds for an incremental PID.
         *
         */
        struct pid_config
        {
                float kp;
                float ki;
                float kd;
                float period;
                float minimum;
                float maximum;
        };

        /**
         * @brief Store the error history and bounded controller output.
         *
         */
        class incremental_pid
        {
                public:
                        explicit incremental_pid(const pid_config& config);
                        void configure(const pid_config& config);
                        void reset(float command, float error);
                        float update(float target, float measured, float feedforward_delta = 0.0f);

                private:
                        float clamp(float command) const;

                        float _kp;
                        float _ki_step;
                        float _kd_step;
                        float _minimum;
                        float _maximum;
                        float _command;
                        float _previous;
                        float _older;
        };
}
