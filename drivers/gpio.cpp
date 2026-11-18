/** @file gpio.cpp
 *  @brief HAL GPIO pin configuration and access.
 */
#include "gpio.hpp"

#include "board.hpp"

namespace
{
    /** @brief Apply a GPIO configuration to one or more pins on an enabled port. */
    void configure(GPIO_TypeDef* port, uint16_t pins, uint32_t mode, uint32_t pull, uint32_t af)
    {
        GPIO_InitTypeDef config{};
        config.Pin = pins;
        config.Mode = mode;
        config.Pull = pull;
        config.Speed = GPIO_SPEED_FREQ_HIGH;
        config.Alternate = af;
        HAL_GPIO_Init(port, &config);
    }
}

namespace drivers
{
    /** @brief Set an initial pin level before selecting push-pull output mode. */
    void gpio::output(GPIO_TypeDef* port, uint16_t pins, bool initial)
    {
        write(port, pins, initial);
        configure(port, pins, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, 0);
    }

    /** @brief Select analog mode with no internal pull. */
    void gpio::analog(GPIO_TypeDef* port, uint16_t pins)
    {
        configure(port, pins, GPIO_MODE_ANALOG, GPIO_NOPULL, 0);
    }

    /** @brief Select an alternate function for the given pins. */
    void gpio::alternate(GPIO_TypeDef* port, uint16_t pins, uint32_t af, uint32_t pull)
    {
        configure(port, pins, GPIO_MODE_AF_PP, pull, af);
    }

    /** @brief Set or clear all pins in the mask using the atomic GPIO set/reset register. */
    void gpio::write(GPIO_TypeDef* port, uint16_t pins, bool high)
    {
        HAL_GPIO_WritePin(port, pins, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    /** @brief Return true when at least one pin in the mask reads high. */
    bool gpio::read(GPIO_TypeDef* port, uint16_t pins)
    {
        return HAL_GPIO_ReadPin(port, pins) == GPIO_PIN_SET;
    }

    /** @brief Toggle selected output pins atomically with respect to local interrupts. */
    void gpio::toggle(GPIO_TypeDef* port, uint16_t pins)
    {
        board::critical_section lock;
        HAL_GPIO_TogglePin(port, pins);
    }
}
