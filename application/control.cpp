/** @file control.cpp
 * @brief Fixed-rate control input and foreground communications.
 */
#include "control.hpp"

#include <cstdio>

#include "adc.hpp"
#include "board.hpp"
#include "board_options.hpp"
#include "can.hpp"
#include "gpio.hpp"
#include "hrtim.hpp"
#include "uart.hpp"

namespace
{
    constexpr uint32_t period_cycles = board::CoreHz / board::ControlHz;
    volatile uint32_t windows;
    volatile uint32_t missed;
    volatile uint32_t max_cycles;
    drivers::control_sample input{};
    char stdout_buffer[256];
}

/** @brief Consume one input window every fourth PWM period, including while gates are disabled. */
extern "C" void HRTIM1_Master_IRQHandler()
{
    auto& master = HRTIM1->sMasterRegs;
    if ((master.MISR & HRTIM_MISR_MREP) == 0)
    {
        return;
    }
    master.MICR = HRTIM_MICR_MREP;
    const uint32_t begin = DWT->CYCCNT;
    static uint32_t previous;
    const uint32_t elapsed = begin - previous;
    previous = begin;
    if (windows != 0 && elapsed > period_cycles + period_cycles / 2U)
    {
        drivers::adc::instance().fail();
    }
    ++windows;
    if (!drivers::adc::instance().readWindow(windows, input))
    {
        ++missed;
    }
    // The validated input is available here for the future closed-loop law.
    const uint32_t cycles = DWT->CYCCNT - begin;
    if (cycles > max_cycles)
    {
        max_cycles = cycles;
    }
    if ((master.MISR & HRTIM_MISR_MREP) != 0)
    {
        drivers::adc::instance().fail();
    }
}

namespace application
{
    /** @brief Arm both DMA receivers before starting the hardware window clock. */
    [[noreturn]] void run()
    {
        if (!board::init() || !drivers::hrtim::instance().init() || !drivers::uart::instance().init() ||
            !drivers::adc::instance().init() ||
            (board::can_wiring_confirmed && !drivers::can::instance().init()) ||
            !drivers::hrtim::instance().startTimebase())
        {
            board::panic();
        }
        setvbuf(stdout, stdout_buffer, _IOLBF, sizeof(stdout_buffer));
        std::printf("SuperCap 31250 Hz, PWM outputs disabled\r\n");
        uint32_t report = HAL_GetTick();
        while (true)
        {
            drivers::uart::instance().service();
            drivers::can_frame frame{};
            for (unsigned count = 0; count < 15 && drivers::can::instance().receive(frame); ++count)
            {
                drivers::gpio::write(GPIOB, board::LedBlue, true);
            }
            if (HAL_GetTick() - report >= 1000U)
            {
                report = HAL_GetTick();
                uint32_t sequence;
                uint32_t lost;
                uint32_t worst;
                drivers::control_sample snapshot;
                {
                    board::critical_section lock;
                    sequence = windows;
                    lost = missed;
                    worst = max_cycles;
                    snapshot = input;
                }
                clearerr(stdout);
                std::printf("window=%lu missed=%lu max_cycles=%lu Icap=%u Ibus_avg=%u\r\n",
                            static_cast<unsigned long>(sequence), static_cast<unsigned long>(lost),
                            static_cast<unsigned long>(worst), static_cast<unsigned>(snapshot.last.capacitor_current),
                            static_cast<unsigned>(snapshot.mean_chassis_current));
            }
            __WFI();
        }
    }
}
