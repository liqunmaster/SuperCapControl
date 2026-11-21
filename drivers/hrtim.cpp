/**
 * @file hrtim.cpp
 * @brief HRTIM1 PWM and paired memory-to-compare DMA transactions.
 */
#include "hrtim.hpp"

#include "gpio.hpp"

namespace
{
        constexpr uint64_t timer_hz        = static_cast<uint64_t>(board::CoreHz) * 32U;
        constexpr uint32_t pwm_period      = static_cast<uint32_t>(timer_hz / board::PwmHz);
        constexpr uint32_t dead_time_ticks = static_cast<uint32_t>(
                (static_cast<uint64_t>(board::PwmDeadTimeNs) * 8U * board::CoreHz + 999999999U) / 1000000000U);
        constexpr uint32_t outputs         = HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2;
        constexpr uint32_t counters    = HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B;
        constexpr uint32_t update_mask = HRTIM_CR1_TAUDIS | HRTIM_CR1_TBUDIS;
        constexpr uint32_t a = HRTIM_TIMERINDEX_TIMER_A;
        constexpr uint32_t b = HRTIM_TIMERINDEX_TIMER_B;

        static_assert(timer_hz % board::PwmHz == 0 && pwm_period <= UINT16_MAX, "Invalid HRTIM period");
        static_assert(dead_time_ticks <= 0x1FF && dead_time_ticks * 8U < pwm_period / 20U, "Invalid dead time");

        /**
         * @brief Validate a run duty in per mille.
         *
         */
        bool validDuty(uint16_t duty)
        {
                return duty >= 50 && duty <= 950;
        }

        /**
         * @brief Configure one HRTIM compare DMA channel for a single word per transaction.
         *
         */
        bool configureDma(DMA_HandleTypeDef& dma, DMA_Channel_TypeDef* channel, uint32_t request)
        {
                dma.Instance = channel;
                dma.Init = {request, DMA_MEMORY_TO_PERIPH, DMA_PINC_DISABLE, DMA_MINC_DISABLE,
                                    DMA_PDATAALIGN_WORD, DMA_MDATAALIGN_WORD, DMA_NORMAL, DMA_PRIORITY_VERY_HIGH};
                return HAL_DMA_Init(&dma) == HAL_OK;
        }

        /**
         * @brief Set PWM edges and inactive levels for one output.
         *
         */
        bool configureOutput(HRTIM_HandleTypeDef& timer, uint32_t index, uint32_t output, bool high_side)
        {
                HRTIM_OutputCfgTypeDef pin{};
                pin.Polarity    = HRTIM_OUTPUTPOLARITY_HIGH;
                pin.SetSource   = high_side ? HRTIM_OUTPUTSET_TIMPER : HRTIM_OUTPUTSET_TIMCMP1;
                pin.ResetSource = high_side ? HRTIM_OUTPUTRESET_TIMCMP1 : HRTIM_OUTPUTRESET_TIMPER;
                pin.IdleMode    = HRTIM_OUTPUTIDLEMODE_IDLE;
                pin.IdleLevel   = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
                pin.FaultLevel  = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
                return HAL_HRTIM_WaveformOutputConfig(&timer, index, output, &pin) == HAL_OK;
        }

        /**
         * @brief Configure both timing units to latch at the same master period boundary.
         *
         */
        bool configureTimer(HRTIM_HandleTypeDef& timer, uint32_t index)
        {
                HRTIM_TimeBaseCfgTypeDef timebase{};
                timebase.Period = pwm_period;
                timebase.PrescalerRatio = index == HRTIM_TIMERINDEX_MASTER ? HRTIM_PRESCALERRATIO_MUL8 :
                                                                                    HRTIM_PRESCALERRATIO_MUL32;
                timebase.Mode = HRTIM_MODE_CONTINUOUS;
                if (HAL_HRTIM_TimeBaseConfig(&timer, index, &timebase) != HAL_OK) {
                        return false;
                }

                HRTIM_TimerCfgTypeDef config{};
                config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
                if (index == HRTIM_TIMERINDEX_MASTER) {
                        config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
                        return HAL_HRTIM_WaveformTimerConfig(&timer, index, &config) == HAL_OK;
                }

                config.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_MASTER;
                config.ResetTrigger  = HRTIM_TIMRESETTRIGGER_MASTER_PER;
                config.DeadTimeInsertion = dead_time_ticks == 0 ? HRTIM_TIMDEADTIMEINSERTION_DISABLED :
                                                                        HRTIM_TIMDEADTIMEINSERTION_ENABLED;
                HRTIM_DeadTimeCfgTypeDef dead_time{};
                dead_time.Prescaler    = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
                dead_time.RisingValue  = dead_time_ticks;
                dead_time.FallingValue = dead_time_ticks;
                return HAL_HRTIM_WaveformTimerConfig(&timer, index, &config) == HAL_OK &&
                              HAL_HRTIM_DeadTimeConfig(&timer, index, &dead_time) == HAL_OK;
        }

        /**
         * @brief Send one DMA word to the timer's compare-1 preload register.
         *
         */
        bool armCompare(DMA_HandleTypeDef& dma, const uint32_t& source, volatile uint32_t& destination)
        {
                return HAL_DMA_Start_IT(&dma, reinterpret_cast<uint32_t>(&source),
                                                reinterpret_cast<uint32_t>(&destination), 1) == HAL_OK;
        }

        /**
         * @brief Route DMA completion to the bridge that owns the channel.
         *
         */
        void onDmaComplete(DMA_HandleTypeDef* dma)
        {
                drivers::hrtim::instance().complete(dma->Instance == DMA1_Channel2);
        }

        /**
         * @brief Latch all PWM outputs off after an error on either bridge DMA.
         *
         */
        void onDmaError(DMA_HandleTypeDef*)
        {
                drivers::hrtim::disableOutputs();
        }
}

namespace drivers
{
        /**
         * @brief Acknowledge a pending Master repetition event.
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
         * @brief Check whether the next hardware control tick has arrived.
         *
         */
        bool hrtim::controlTickPending()
        {
                return (HRTIM1->sMasterRegs.MISR & HRTIM_MISR_MREP) != 0;
        }

        /**
         * @brief Return the sole owner of HRTIM1 and DMA1 channels 1/2.
         *
         */
        hrtim& hrtim::instance()
        {
                static hrtim device;
                return device;
        }

        /**
         * @brief Configure the bench waveform and DMA while all gate-command pins are low.
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
                        !configureOutput(_timer, b, HRTIM_OUTPUT_TB2, true) ||
                        !configureDma(_dma_a, DMA1_Channel1, DMA_REQUEST_HRTIM1_A) ||
                        !configureDma(_dma_b, DMA1_Channel2, DMA_REQUEST_HRTIM1_B)) {
                        disableOutputs();
                        return false;
                }

                _dma_a.XferCpltCallback = onDmaComplete;
                _dma_b.XferCpltCallback = onDmaComplete;
                _dma_a.XferErrorCallback = onDmaError;
                _dma_b.XferErrorCallback = onDmaError;
                HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, board::PwmDmaPriority, 0);
                HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, board::PwmDmaPriority, 0);
                HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
                HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
                _ready = true;
                return true;
        }

        /**
         * @brief Start the 32 us sampling window with both bridge outputs disabled.
         *
         */
        bool hrtim::startTimebase()
        {
                static_assert(board::PwmHz == 4 * board::ControlHz, "Four PWM cycles per control window");
                auto& master = HRTIM1->sMasterRegs;
                const uint32_t phase = pwm_period / 64U;
                master.MCMP1R = phase;
                master.MCMP2R = pwm_period / 4U + phase;
                master.MCMP3R = pwm_period / 2U + phase;
                master.MCMP4R = 3U * pwm_period / 4U + phase;
                HRTIM_ADCTriggerCfgTypeDef trigger{};
                trigger.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
                trigger.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP1 | HRTIM_ADCTRIGGEREVENT13_MASTER_CMP2 |
                                          HRTIM_ADCTRIGGEREVENT13_MASTER_CMP3 | HRTIM_ADCTRIGGEREVENT13_MASTER_CMP4;
                if (HAL_HRTIM_ADCTriggerConfig(&_timer, HRTIM_ADCTRIGGER_1, &trigger) != HAL_OK) {
                        return false;
                }
                trigger.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP4;
                if (HAL_HRTIM_ADCTriggerConfig(&_timer, HRTIM_ADCTRIGGER_3, &trigger) != HAL_OK ||
                        HAL_HRTIM_SoftwareUpdate(&_timer, HRTIM_TIMERUPDATE_MASTER) != HAL_OK) {
                        return false;
                }
                master.MICR = HRTIM_MICR_MREP;
                master.MDIER |= HRTIM_MASTER_IT_MREP;
                HAL_NVIC_SetPriority(HRTIM1_Master_IRQn, board::AdcDmaPriority, 0);
                HAL_NVIC_EnableIRQ(HRTIM1_Master_IRQn);
                return HAL_HRTIM_WaveformCountStart(&_timer, HRTIM_TIMERID_MASTER) == HAL_OK;
        }

        /**
         * @brief Explicitly start both bridges from known initial compare values.
         * @param duty_a Bridge A high-side duty, in per mille from 50 to 950.
         * @param duty_b Bridge B high-side duty, in per mille from 50 to 950.
         * @return False for invalid duties, an uninitialized/faulted driver or an active waveform.
         * @pre Call from the PWM owning task. Verify isolated hardware before enabling outputs.
         *
         */
        bool hrtim::start(uint16_t duty_a, uint16_t duty_b)
        {
                board::critical_section lock;
                if (!_ready || _fault || _running || !validDuty(duty_a) || !validDuty(duty_b)) {
                        return false;
                }

                // The sampling master continues independently of bridge enables.
                HRTIM1->sTimerxRegs[a].CNTxR = 0;
                HRTIM1->sTimerxRegs[b].CNTxR = 0;
                HRTIM1->sTimerxRegs[a].CMP1xR = pwm_period * duty_a / 1000U;
                HRTIM1->sTimerxRegs[b].CMP1xR = pwm_period * duty_b / 1000U;
                if (HAL_HRTIM_SoftwareUpdate(&_timer, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, a, HRTIM_OUTPUT_TA1,
                                                         HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, a, HRTIM_OUTPUT_TA2,
                                                         HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, b, HRTIM_OUTPUT_TB1,
                                                         HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK ||
                        HAL_HRTIM_WaveformSetOutputLevel(&_timer, b, HRTIM_OUTPUT_TB2,
                                                         HRTIM_OUTPUTLEVEL_INACTIVE) != HAL_OK) {
                        disableOutputs();
                        return false;
                }

                gpio::alternate(GPIOA, board::PwmPins, GPIO_AF13_HRTIM1, GPIO_PULLDOWN);
                if (HAL_HRTIM_WaveformOutputStart(&_timer, outputs) != HAL_OK ||
                        HAL_HRTIM_WaveformCountStart(&_timer, counters) != HAL_OK) {
                        disableOutputs();
                        return false;
                }
                _running = true;
                return true;
        }

        /**
         * @brief Queue one paired duty update, transferred by DMA on hardware counter reset events.
         * @param duty_a Bridge A high-side duty, in per mille from 50 to 950.
         * @param duty_b Bridge B high-side duty, in per mille from 50 to 950.
         * @return True if both DMA channels were armed. A busy or disabled driver returns false.
         * @pre Call from the PWM owning task.
         *
         */
        bool hrtim::update(uint16_t duty_a, uint16_t duty_b)
        {
                if (!validDuty(duty_a) || !validDuty(duty_b)) {
                        return false;
                }

                board::critical_section lock;
                if (!_running || _pending != 0 || (_await_latch && (HRTIM1->sMasterRegs.MISR & HRTIM_MISR_MUPD) == 0)) {
                        return false;
                }

                HRTIM1->sCommonRegs.CR1 |= update_mask;
                _await_latch = false;
                _compare_a = pwm_period * duty_a / 1000U;
                _compare_b = pwm_period * duty_b / 1000U;
                _pending = 3U;
                __DMB();
                if (!armCompare(_dma_a, _compare_a, HRTIM1->sTimerxRegs[a].CMP1xR) ||
                        !armCompare(_dma_b, _compare_b, HRTIM1->sTimerxRegs[b].CMP1xR)) {
                        disableOutputs();
                        return false;
                }

                HRTIM1->sTimerxRegs[a].TIMxDIER |= HRTIM_TIM_DMA_RST;
                HRTIM1->sTimerxRegs[b].TIMxDIER |= HRTIM_TIM_DMA_RST;
                return true;
        }

        /**
         * @brief Release both preload gates only once both DMA writes have completed.
         *
         */
        void hrtim::complete(bool second)
        {
                board::critical_section lock;
                if (!_running) {
                        return;
                }

                HRTIM1->sTimerxRegs[second ? b : a].TIMxDIER &= ~HRTIM_TIM_DMA_RST;
                _pending &= ~(second ? 2U : 1U);
                if (_pending == 0) {
                        __DMB();
                        HRTIM1->sCommonRegs.CR1 &= ~update_mask;
                        HRTIM1->sMasterRegs.MICR = HRTIM_MASTER_FLAG_MUPD;
                        _await_latch = true;
                }
        }

        /**
         * @brief Cancel pending requests and release DMA state during shutdown.
         *
         */
        void hrtim::cancelUpdate()
        {
                HRTIM1->sTimerxRegs[a].TIMxDIER &= ~HRTIM_TIM_DMA_RST;
                HRTIM1->sTimerxRegs[b].TIMxDIER &= ~HRTIM_TIM_DMA_RST;
                if (_dma_a.State == HAL_DMA_STATE_BUSY) {
                        HAL_DMA_Abort(&_dma_a);
                }
                if (_dma_b.State == HAL_DMA_STATE_BUSY) {
                        HAL_DMA_Abort(&_dma_b);
                }
                HRTIM1->sCommonRegs.CR1 &= ~update_mask;
                _pending = 0;
                _await_latch = false;
        }

        /**
         * @brief Disable outputs before stopping timers and changing pin mode.
         *
         */
        void hrtim::stopHardware()
        {
                _running = false;
                if (__HAL_RCC_HRTIM1_IS_CLK_ENABLED()) {
                        HRTIM1->sCommonRegs.ODISR = outputs;
                        HRTIM1->sMasterRegs.MCR &= ~(HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
                        cancelUpdate();
                }
                if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
                        gpio::output(GPIOA, board::PwmPins, false);
                }
        }

        /**
         * @brief Stop the waveform and permit a later explicit start.
         *
         */
        void hrtim::stop()
        {
                board::critical_section lock;
                stopHardware();
        }

        /**
         * @brief Latch shutdown; subsequent start calls fail until a reset.
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
         * @brief Return whether the waveform has been explicitly started.
         *
         */
        bool hrtim::running() const
        {
                return _running;
        }

        /**
         * @brief Return the latched fault state.
         *
         */
        bool hrtim::faulted() const
        {
                return _fault;
        }

        /**
         * @brief Dispatch the bridge DMA IRQ and its completion/error callbacks.
         *
         */
        void hrtim::handleDmaIrq(bool second)
        {
                const uint32_t error = second ? DMA_ISR_TEIF2 : DMA_ISR_TEIF1;
                if (!_running) {
                        DMA1->IFCR = second ? DMA_IFCR_CGIF2 : DMA_IFCR_CGIF1;
                        return;
                }
                if ((DMA1->ISR & error) != 0) {
                        disableOutputs();
                        return;
                }
                HAL_DMA_IRQHandler(second ? &_dma_b : &_dma_a);
        }
}

/**
 * @brief Dispatch the HRTIM timer A compare DMA interrupt.
 *
 */
extern "C" void DMA1_Channel1_IRQHandler()
{
        drivers::hrtim::instance().handleDmaIrq(false);
}

/**
 * @brief Dispatch the HRTIM timer B compare DMA interrupt.
 *
 */
extern "C" void DMA1_Channel2_IRQHandler()
{
        drivers::hrtim::instance().handleDmaIrq(true);
}
