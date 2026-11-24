/**
 * @file hrtim.cpp
 * @brief HRTIM1 双桥 PWM 输出与比较寄存器的成对预装载更新。
 */
#include "hrtim.hpp"

#include "gpio.hpp"

namespace
{
        constexpr uint32_t pwm_period      = board::pwm_timing::period_ticks;
        constexpr uint32_t dead_time_ticks = static_cast<uint32_t>((static_cast<uint64_t>(board::PwmDeadTimeNs) * 8U * board::CoreHz + 999999999U) / 1000000000U);
        constexpr uint32_t outputs         = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2;
        constexpr uint32_t counters        = HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B;
        constexpr uint32_t update_mask     = HRTIM_CR1_TAUDIS | HRTIM_CR1_TBUDIS;
        constexpr uint32_t a               = HRTIM_TIMERINDEX_TIMER_A;
        constexpr uint32_t b               = HRTIM_TIMERINDEX_TIMER_B;

        static_assert(pwm_period >= 96U && pwm_period <= 0xFFDFU, "Invalid HRTIM MUL32 period");
        static_assert(dead_time_ticks <= 0x1FF && dead_time_ticks * 8U < pwm_period / 20U, "Invalid dead time");

        /**
         * @brief 检查上管占空比是否位于允许的千分比范围内。
         *
         */
        bool validDuty(uint16_t duty)
        {
                return duty >= 50 && duty <= 950;
        }

        /**
         * @brief 配置一路 PWM 的置位、复位事件及空闲电平。
         *
         */
        bool configureOutput(HRTIM_HandleTypeDef& timer, uint32_t index, uint32_t output, bool high_side)
        {
                HRTIM_OutputCfgTypeDef pin{};
                pin.Polarity    = HRTIM_OUTPUTPOLARITY_HIGH;
                pin.SetSource   = high_side ? HRTIM_OUTPUTSET_TIMCMP3 : HRTIM_OUTPUTSET_TIMCMP4;
                pin.ResetSource = high_side ? HRTIM_OUTPUTRESET_TIMCMP4 : HRTIM_OUTPUTRESET_TIMCMP3;
                pin.IdleMode    = HRTIM_OUTPUTIDLEMODE_IDLE;
                pin.IdleLevel   = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
                pin.FaultLevel  = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
                return HAL_HRTIM_WaveformOutputConfig(&timer, index, output, &pin) == HAL_OK;
        }

        /**
         * @brief 配置两桥计数器，使比较预装载值在同一主周期边界生效。
         *
         */
        bool configureTimer(HRTIM_HandleTypeDef& timer, uint32_t index)
        {
                HRTIM_TimeBaseCfgTypeDef timebase{};
                timebase.Period         = pwm_period;
                timebase.PrescalerRatio = index == HRTIM_TIMERINDEX_MASTER ? HRTIM_PRESCALERRATIO_MUL8 : HRTIM_PRESCALERRATIO_MUL32;
                timebase.Mode           = HRTIM_MODE_CONTINUOUS;
                if (HAL_HRTIM_TimeBaseConfig(&timer, index, &timebase) != HAL_OK) {
                        return false;
                }

                HRTIM_TimerCfgTypeDef config{};
                config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
                if (index == HRTIM_TIMERINDEX_MASTER) {
                        config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
                        return HAL_HRTIM_WaveformTimerConfig(&timer, index, &config) == HAL_OK;
                }

                config.UpdateTrigger     = HRTIM_TIMUPDATETRIGGER_MASTER;
                config.ResetTrigger      = HRTIM_TIMRESETTRIGGER_MASTER_PER;
                config.DeadTimeInsertion = dead_time_ticks == 0 ? HRTIM_TIMDEADTIMEINSERTION_DISABLED : HRTIM_TIMDEADTIMEINSERTION_ENABLED;
                HRTIM_DeadTimeCfgTypeDef dead_time{};
                dead_time.Prescaler    = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
                dead_time.RisingValue  = dead_time_ticks;
                dead_time.FallingValue = dead_time_ticks;
                return HAL_HRTIM_WaveformTimerConfig(&timer, index, &config) == HAL_OK && HAL_HRTIM_DeadTimeConfig(&timer, index, &dead_time) == HAL_OK;
        }

        /**
         * @brief 计算居中 PWM 的两个开关边沿。
         *
         */
        void setCompare(uint32_t (&compare)[2], uint16_t duty)
        {
                const uint32_t half_width = pwm_period * duty / 2000U;
                compare[0] = pwm_period / 2U - half_width;
                compare[1] = pwm_period / 2U + half_width;
        }


}

namespace drivers
{
        /**
         * @brief 检查并清除主定时器重复事件标志。
         *
         */
        bool hrtim::takeControlTick()
        {
                if (!controlTickPending()) {
                        return false;
                }
                HRTIM1->sMasterRegs.MICR = HRTIM_MICR_MREP;
                return true;
        }

        /**
         * @brief 检查下一次主定时器控制节拍是否已经到达。
         *
         */
        bool hrtim::controlTickPending()
        {
                return (HRTIM1->sMasterRegs.MISR & HRTIM_MISR_MREP) != 0;
        }

        /**
         * @brief 获取管理 HRTIM1 的驱动实例。
         *
         */
        hrtim& hrtim::instance()
        {
                static hrtim device;
                return device;
        }

        /**
         * @brief 保持门极命令为低，初始化波形发生器。
         *
         */
        bool hrtim::init()
        {
                __HAL_RCC_HRTIM1_CLK_ENABLE();
                HRTIM1->sCommonRegs.ODISR = outputs;
                _timer.Instance = HRTIM1;
                if (HAL_HRTIM_Init(&_timer) != HAL_OK ||
                        HAL_HRTIM_DLLCalibrationStart(&_timer, HRTIM_CALIBRATIONRATE_3) != HAL_OK ||
                        HAL_HRTIM_PollForDLLCalibration(&_timer, 10) != HAL_OK ||
                        !configureTimer(_timer, HRTIM_TIMERINDEX_MASTER) || !configureTimer(_timer, a) ||
                        !configureTimer(_timer, b) || !configureOutput(_timer, a, HRTIM_OUTPUT_TA1, true) ||
                        !configureOutput(_timer, a, HRTIM_OUTPUT_TA2, false) ||
                        !configureOutput(_timer, b, HRTIM_OUTPUT_TB1, false) ||
                        !configureOutput(_timer, b, HRTIM_OUTPUT_TB2, true) ) {
                        disableOutputs();
                        return false;
                }

                _ready = true;
                return true;
        }

        /**
         * @brief 在两桥输出关闭时启动每四个 PWM 周期的采样窗口。
         *
         */
        bool hrtim::startTimebase()
        {
                using timing = board::adc_timing;
                constexpr uint32_t ticks_per_half_clock = board::pwm_timing::bridge_multiplier * timing::clock_divider / 2U;
                constexpr uint32_t acquisition_ticks    = timing::acquisition_halves * ticks_per_half_clock;
                constexpr uint32_t rank_ticks           = acquisition_ticks + timing::conversion_halves * ticks_per_half_clock;
                constexpr uint32_t adc2_lead            = timing::capacitor_current_index * rank_ticks;
                constexpr uint32_t centered_start      = (pwm_period - acquisition_ticks) / 2U;
                // ADC2 先采两个通道，两路电流采样一起后移，给周期起点留出触发余量。
                constexpr uint32_t earliest_adc2_trigger = 8U * board::pwm_timing::bridge_multiplier;
                constexpr uint32_t earliest_current_start = adc2_lead + earliest_adc2_trigger;
                constexpr uint32_t current_sample_start = centered_start < earliest_current_start ? earliest_current_start : centered_start;
                constexpr uint32_t adc2_trigger = current_sample_start - adc2_lead;
                static_assert(current_sample_start + timing::adc1_channels * rank_ticks < pwm_period, "ADC1 scan exceeds one PWM cycle");
                static_assert(adc2_trigger + timing::adc2_channels * rank_ticks < pwm_period, "ADC2 scan exceeds one PWM cycle");

                auto& master = HRTIM1->sMasterRegs;
                HRTIM1->sTimerxRegs[a].CMP2xR = current_sample_start;
                HRTIM1->sTimerxRegs[b].CMP2xR = adc2_trigger;
                HRTIM_ADCTriggerCfgTypeDef trigger{};
                trigger.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
                trigger.Trigger = HRTIM_ADCTRIGGEREVENT6810_TIMERA_CMP2;
                if (HAL_HRTIM_ADCTriggerConfig(&_timer, HRTIM_ADCTRIGGER_6, &trigger) != HAL_OK) {
                        return false;
                }
                trigger.Trigger = HRTIM_ADCTRIGGEREVENT6810_TIMERB_CMP2;
                if (HAL_HRTIM_ADCTriggerConfig(&_timer, HRTIM_ADCTRIGGER_8, &trigger) != HAL_OK || HAL_HRTIM_SoftwareUpdate(&_timer, HRTIM_TIMERUPDATE_MASTER | HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK) {
                        return false;
                }
                master.MICR = HRTIM_MICR_MREP;
                master.MDIER |= HRTIM_MASTER_IT_MREP;
                HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, board::AdcDmaPriority, 0);
                HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
                return HAL_HRTIM_WaveformCountStart(&_timer, counters) == HAL_OK;
        }

        /**
         * @brief 在公共更新边界启用两桥的初始占空比。
         * @param duty_a A 桥上管占空比，单位为千分比，范围为 50 至 950。
         * @param duty_b B 桥上管占空比，单位为千分比，范围为 50 至 950。
         * @return 成功返回 true。参数无效、驱动未就绪、已启动或故障时返回 false。
         * @pre 由 PWM 控制端调用，使能前须在隔离功率级条件下核对波形。
         *
         */
        bool hrtim::start(uint16_t duty_a, uint16_t duty_b)
        {
                board::critical_section lock;
                if (!_ready || _fault || _running || !validDuty(duty_a) || !validDuty(duty_b)) {
                        return false;
                }

                // 屏蔽本周期的比较边沿，先将两个开关节点置于低侧续流状态。
                constexpr uint32_t parked_compare = pwm_period + 32U;
                static_assert(parked_compare <= 0xFFDFU, "Parked compare exceeds the HRTIM MUL32 range");
                HRTIM1->sTimerxRegs[a].CMP3xR = parked_compare;
                HRTIM1->sTimerxRegs[a].CMP4xR = parked_compare;
                HRTIM1->sTimerxRegs[b].CMP3xR = parked_compare;
                HRTIM1->sTimerxRegs[b].CMP4xR = parked_compare;
                if (HAL_HRTIM_SoftwareUpdate(&_timer, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, a, HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, a, HRTIM_OUTPUT_TA2, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, b, HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, b, HRTIM_OUTPUT_TB2, HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) {
                        disableOutputs();
                        return false;
                }

                gpio::alternate(GPIOA, board::PwmPins, GPIO_AF13_HRTIM1, GPIO_PULLDOWN);
                if (HAL_HRTIM_WaveformOutputStart(&_timer, outputs) != HAL_OK) {
                        disableOutputs();
                        return false;
                }
                // 四个边沿在同一个主周期边界生效，避免启动半个脉冲。
                HRTIM1->sCommonRegs.CR1 |= update_mask;
                setCompare(_compare_a, duty_a);
                setCompare(_compare_b, duty_b);
                HRTIM1->sTimerxRegs[a].CMP3xR = _compare_a[0];
                HRTIM1->sTimerxRegs[a].CMP4xR = _compare_a[1];
                HRTIM1->sTimerxRegs[b].CMP3xR = _compare_b[0];
                HRTIM1->sTimerxRegs[b].CMP4xR = _compare_b[1];
                releaseCompareUpdate();
                _running = true;
                return true;
        }

        /**
         * @brief 提交两桥的新占空比，在公共更新边界生效。
         * @param duty_a A 桥上管占空比，单位为千分比，范围为 50 至 950。
         * @param duty_b B 桥上管占空比，单位为千分比，范围为 50 至 950。
         * @return 已写入两桥预装载时返回 true。参数无效、忙或输出关闭时返回 false。
         * @pre 由 PWM 控制端调用。
         *
         */
        bool hrtim::update(uint16_t duty_a, uint16_t duty_b)
        {
                if (!validDuty(duty_a) || !validDuty(duty_b)) {
                        return false;
                }

                board::critical_section lock;
                if (!_running || (_await_latch && (HRTIM1->sMasterRegs.MISR & HRTIM_MISR_MUPD) == 0)) {
                        return false;
                }

                HRTIM1->sCommonRegs.CR1 |= update_mask;
                _await_latch = false;
                setCompare(_compare_a, duty_a);
                setCompare(_compare_b, duty_b);
                HRTIM1->sTimerxRegs[a].CMP3xR = _compare_a[0];
                HRTIM1->sTimerxRegs[a].CMP4xR = _compare_a[1];
                HRTIM1->sTimerxRegs[b].CMP3xR = _compare_b[0];
                HRTIM1->sTimerxRegs[b].CMP4xR = _compare_b[1];
                releaseCompareUpdate();
                return true;
        }

        /**
         * @brief 同时开放两桥预装载，等待公共更新事件后再接受下一组占空比。
         *
         */
        void hrtim::releaseCompareUpdate()
        {
                __DMB();
                HRTIM1->sCommonRegs.CR1 &= ~update_mask;
                HRTIM1->sMasterRegs.MICR = HRTIM_MASTER_FLAG_MUPD;
                _await_latch = true;
        }

        /**
         * @brief 关闭输出时取消待处理的更新。
         *
         */
        void hrtim::cancelUpdate()
        {
                HRTIM1->sCommonRegs.CR1 &= ~update_mask;
                _await_latch = false;
        }

        /**
         * @brief 关断门极输出并取消 PWM 更新，保留 ADC 触发计数器。
         *
         */
        void hrtim::stopHardware()
        {
                _running = false;
                if (__HAL_RCC_HRTIM1_IS_CLK_ENABLED()) {
                        HRTIM1->sCommonRegs.ODISR = outputs;
                        cancelUpdate();
                }
                if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
                        gpio::output(GPIOA, board::PwmPins, false);
                }
        }

        /**
         * @brief 停止 PWM，允许随后显式重启。
         *
         */
        void hrtim::stop()
        {
                board::critical_section lock;
                stopHardware();
        }

        /**
         * @brief 锁存故障并关断，复位后才能重新启动。
         *
         */
        void hrtim::disableOutputs()
        {
                board::critical_section lock;
                auto& driver = instance();
                driver._fault = true;
                driver.stopHardware();
        }

        /**
         * @brief 返回 PWM 输出是否已启动。
         *
         */
        bool hrtim::running() const
        {
                return _running;
        }

        /**
         * @brief 返回故障是否已锁存。
         *
         */
        bool hrtim::faulted() const
        {
                return _fault;
        }

}
