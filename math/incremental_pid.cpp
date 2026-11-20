/**
 * @file incremental_pid.cpp
 * @brief Bounded incremental PID calculations.
 */
#include "incremental_pid.hpp"

namespace math
{
        /**
         * @brief Configure a fixed-period controller and clear its history.
         * @param config Gains, positive period and ordered output bounds.
         *
         */
        incremental_pid::incremental_pid(const pid_config& config)
        {
                configure(config);
        }

        /**
         * @brief Apply new gains and bounds, then reset the output and error history.
         * @param config Gains, positive period and ordered output bounds.
         *
         */
        void incremental_pid::configure(const pid_config& config)
        {
                _kp = config.kp;
                _ki_step = config.ki * config.period;
                _kd_step = config.kd / config.period;
                _minimum = config.minimum;
                _maximum = config.maximum;
                reset(0.0f, 0.0f);
        }

        /**
         * @brief Seed the controller from the applied command and current control error.
         * @param command Initial output, clamped to the configured bounds.
         * @param error Setpoint minus feedback.
         *
         */
        void incremental_pid::reset(float command, float error)
        {
                _command = clamp(command);
                _previous = error;
                _older = error;
        }

        /**
         * @brief Compute one bounded command from finite setpoint and feedback values at the configured interval.
         * @param target Setpoint.
         * @param measured Feedback in the same units as the setpoint.
         * @param feedforward_delta Change in the external feedforward command since the last update.
         * @return Bounded controller output.
         *
         */
        float incremental_pid::update(float target, float measured, float feedforward_delta)
        {
                const float error = target - measured;
                const float slope = error - _previous;
                const float delta = _kp * slope + _ki_step * error + _kd_step * (slope - (_previous - _older));
                _command = clamp(_command + feedforward_delta + delta);
                _older = _previous;
                _previous = error;
                return _command;
        }

        /**
         * @brief Clamp a finite command to the configured interval.
         * @param command Candidate output.
         * @return Output within the configured bounds.
         *
         */
        float incremental_pid::clamp(float command) const
        {
                if (command < _minimum) {
                        return _minimum;
                }
                if (command > _maximum) {
                        return _maximum;
                }
                return command;
        }
}
