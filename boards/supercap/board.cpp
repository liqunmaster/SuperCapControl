/**
 * @brief 板级时钟、GPIO、节拍和故障处理实现。
 * @file board.cpp
 */
#include "board.hpp"

#include "hrtim.hpp"

extern "C"
{
    uint32_t SystemCoreClock = 16000000;  ///< CMSIS 内核时钟频率，复位时为 HSI16。
    /**
     * @brief 配置 FPU 访问权限与向量表，供复位入口调用。
     */
    void SystemInit()
    {
        SCB->CPACR |= 0xFUL << 20;
        SCB->VTOR = FLASH_BASE;
        __DSB();
        __ISB();
    }
    void SystemCoreClockUpdate()
    {
        /// This board has two supported states: reset HSI16 and configured PLL144.
        SystemCoreClock = ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) ? board::CoreHz : 16000000;
    }
}

namespace
{
    volatile uint32_t ticks;
}

namespace board
{
    /**
     * @brief 在有界循环内等待寄存器掩码位达到指定值。
     * @param reg 待检查的寄存器。
     * @param mask 参与比较的位掩码。
     * @param value 期望的掩码后值。
     * @return 匹配返回 true，循环次数耗尽返回 false。
     */
    bool waitBits(volatile uint32_t& reg, uint32_t mask, uint32_t value)
    {
        for (uint32_t remaining = 4000000; remaining != 0; --remaining)
        {
            if ((reg & mask) == value)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 使用 DWT 周期计数器进行短时忙等待。
     * @param us 延迟微秒数，须保证换算后的周期数不溢出 uint32_t。
     * @pre DWT 已启用，SystemCoreClock 与实际时钟一致。
     */
    void delayUs(uint32_t us)
    {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t cycles = us * (SystemCoreClock / 1000000);
        while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles)
        {
            __NOP();
        }
    }

    /**
     * @brief 配置引脚模式、复用及输出属性，先将输出锁存值置低。
     * @param port 已开启时钟的 GPIO 端口。
     * @param pin 引脚编号，范围为 0 至 15。
     * @param mode MODER 编码，范围为 0 至 3。
     * @param af 复用编号，范围为 0 至 15。
     */
    void pinMode(GPIO_TypeDef* port, unsigned pin, unsigned mode, unsigned af)
    {
        port->BSRR = 1UL << (pin + 16);  ///< Establish low before selecting output mode.
        port->OTYPER &= ~(1UL << pin);
        port->PUPDR &= ~(3UL << (pin * 2));
        port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2))) | (2UL << (pin * 2));
        const unsigned bank = pin / 8;
        const unsigned shift = (pin % 8) * 4;
        port->AFR[bank] = (port->AFR[bank] & ~(15UL << shift)) | (af << shift);
        port->MODER = (port->MODER & ~(3UL << (pin * 2))) | (mode << (pin * 2));
    }

    /**
     * @brief 初始化安全 GPIO、144 MHz 时钟、DMA 时钟及毫秒节拍。
     * @return 初始化成功返回 true，时钟切换或外设就绪等待超时返回 false。
     */
    bool init()
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
        (void)RCC->AHB2ENR;
        for (unsigned pin = 8; pin <= 11; ++pin)
        {
            pinMode(GPIOA, pin, 1);
        }
        for (unsigned pin = 12; pin <= 15; ++pin)
        {
            pinMode(GPIOB, pin, 1);
        }
        /// Leave SWD pins PA13/PA14 untouched.
        RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
        PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
        if (!waitBits(PWR->SR2, PWR_SR2_VOSF, 0))
        {
            return false;
        }
        FLASH->ACR = FLASH_ACR_LATENCY_4WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
        if (!waitBits(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_4WS))
        {
            return false;
        }
        RCC->CR |= RCC_CR_HSEON;
        if (!waitBits(RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY))
        {
            return false;
        }
        /// 24 / 6 * 72 / 2 = 144 MHz, range 1 normal mode, APB1/APB2 = 144 MHz.
        RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (5UL << RCC_PLLCFGR_PLLM_Pos) | (72UL << RCC_PLLCFGR_PLLN_Pos) |
                       RCC_PLLCFGR_PLLREN;
        RCC->CR |= RCC_CR_PLLON;
        if (!waitBits(RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY))
        {
            return false;
        }
        /// Intermediate AHB /2 limits the frequency step (RM0440 clock switching).
        RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_SW)) | RCC_CFGR_HPRE_DIV2 | RCC_CFGR_SW_PLL;
        if (!waitBits(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL))
        {
            return false;
        }
        for (volatile unsigned i = 0; i < 256; ++i)
        {
            __NOP();
        }
        RCC->CFGR &= ~RCC_CFGR_HPRE;
        SystemCoreClockUpdate();
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
        (void)RCC->AHB1ENR;
        NVIC_SetPriorityGrouping(3);
        SysTick_Config(SystemCoreClock / 1000);
        NVIC_SetPriority(SysTick_IRQn, 15);
        return true;
    }

    /**
     * @brief 设置 PB12 至 PB15 上的高电平有效 LED。
     * @param pin LED 引脚编号，其他编号忽略。
     * @param on 为 true 时点亮。
     */
    void setLed(unsigned pin, bool on)
    {
        if (pin >= 12 && pin <= 15)
        {
            GPIOB->BSRR = 1UL << (pin + (on ? 0 : 16));
        }
    }

    /**
     * @brief 读取启动后的毫秒计数，溢出后自然回绕。
     * @return 当前毫秒计数。
     */
    uint32_t millis()
    {
        return ticks;
    }

    /**
     * @brief 屏蔽中断、禁用功率输出并进入永久故障循环。
     * @pre GPIOA/GPIOB 时钟已开启。
     */
    [[noreturn]] void panic()
    {
        __disable_irq();
        drivers::hrtim::disableOutputs();
        GPIOA->BSRR = 0x0F00UL << 16;
        setLed(LedRed, true);
        while (true)
        {
            __NOP();
        }
    }
}

/**
 * @brief 更新毫秒节拍。
 */
extern "C" void SysTick_Handler()
{
    ++ticks;
}
/**
 * @brief 处理硬件故障并关闭功率输出。
 */
extern "C" void HardFault_Handler()
{
    board::panic();
}
/**
 * @brief 处理存储器管理故障。
 */
extern "C" void MemManage_Handler()
{
    board::panic();
}
/**
 * @brief 处理总线访问故障。
 */
extern "C" void BusFault_Handler()
{
    board::panic();
}
/**
 * @brief 处理非法指令或用法故障。
 */
extern "C" void UsageFault_Handler()
{
    board::panic();
}
