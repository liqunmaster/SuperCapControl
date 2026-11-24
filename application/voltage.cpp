#include "voltage.hpp"

#include <cmath>

#include "hrtim.hpp"

namespace application
{
        /**
         * @brief 停止 PWM 并清除上一条已生效的命令。
         *
         */
        void voltage_loop::stop()
        {
                auto& pwm = drivers::hrtim::instance();
                if (pwm.running()) {
                        pwm.stop();
                }
                commanded_duty_a = 0;
                commanded_duty_b = 0;
                loop_active = false;
        }

        /**
         * @brief 按母线与电容电压（伏）提交目标电压（毫伏），欠压台架模式只使用前馈。
         *
         */
        void voltage_loop::apply(uint32_t requested, float bus, float measured, bool bench_mode)
        {
                auto& pwm = drivers::hrtim::instance();
                if (pwm.faulted()) {
                        return;
                }
                const float target = static_cast<float>(requested) * 0.001f;
                auto next_pid = voltage_pid;
                float candidate_ratio = target / bus;
                if (!bench_mode) {
                        if (!loop_active) {
                                next_pid.seed(target, measured);
                        }
                        const float base_ratio = loop_active ? accepted_ratio : measured / bus;
                        candidate_ratio = base_ratio + next_pid.update(target, measured) / bus;
                }
                constexpr float min_duty         = 50.0f;
                constexpr float max_duty         = 950.0f;
                constexpr float fixed_duty       = max_duty;
                constexpr float buck_ratio       = 0.8f;
                constexpr float boost_ratio      = 1.25f;
                constexpr float transition_scale = fixed_duty / (1.0f + boost_ratio);
                const float ratio = std::fmin(max_duty / min_duty, std::fmax(min_duty / max_duty, candidate_ratio));
                float duty_a;
                float duty_b;
                if (ratio < buck_ratio) {
                        duty_a = fixed_duty * ratio;
                        duty_b = fixed_duty;
                }
                else if (ratio > boost_ratio) {
                        duty_a = fixed_duty;
                        duty_b = fixed_duty / ratio;
                }
                else {
                        duty_a = transition_scale * (1.0f + ratio);
                        duty_b = transition_scale * (1.0f + 1.0f / ratio);
                }
                const auto next_a    = static_cast<uint16_t>(std::fmin(max_duty, std::fmax(min_duty, duty_a)) + 0.5f);
                const auto next_b    = static_cast<uint16_t>(std::fmin(max_duty, std::fmax(min_duty, duty_b)) + 0.5f);
                const bool running   = pwm.running();
                const bool unchanged = (next_a == commanded_duty_a) && (next_b == commanded_duty_b);
                const bool accepted  = running ? (unchanged || pwm.update(next_a, next_b)) : pwm.start(next_a, next_b);
                if (accepted) {
                        voltage_pid      = next_pid;
                        commanded_duty_a = next_a;
                        commanded_duty_b = next_b;
                        accepted_ratio   = ratio;
                        loop_active      = !bench_mode;
                }
        }
}
