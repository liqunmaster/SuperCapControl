/**
 * @brief FDCAN3 消息 RAM 和中断收发实现。
 * @file can.cpp
 */
#include "can.hpp"

#include "board.hpp"

namespace
{
    /// RM0440 fixed message RAM layout; G4 has 72-byte Rx/Tx elements even for CAN2.0.
    constexpr uint32_t InstanceBytes = 848;
    constexpr uint32_t RxOffset = 144;
    constexpr uint32_t TxOffset = 624;
    constexpr uint32_t ElementBytes = 72;
    volatile uint32_t* element(uint32_t offset, unsigned index)
    {
        return reinterpret_cast<volatile uint32_t*>(SRAMCAN_BASE + 2 * InstanceBytes + offset + index * ElementBytes);
    }
}
namespace drivers
{
    /**
     * @brief 获取唯一 CAN 驱动实例。
     * @return 驱动实例引用。
     */
    can& can::instance()
    {
        static can can;
        return can;
    }

    /**
     * @brief 初始化 FDCAN3，确认物理走线后才允许启动总线。
     * @pre 板级时钟已初始化；仅在启动时调用一次。
     * @return 走线开关开启且外设初始化成功时返回 true，否则返回 false。
     */
    bool can::init()
    {
        if (!SUPERCAP_CAN_WIRING_CONFIRMED)
        {
            return false;
        }
        /// PB3 = RX, PB4 = TX, AF11. Screenshot labels are reversed.
        RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL;  ///< HSE24, independent of APB.
        RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
        (void)RCC->APB1ENR1;
        FDCAN3->CCCR &= ~FDCAN_CCCR_CSR;
        if (!board::waitBits(FDCAN3->CCCR, FDCAN_CCCR_CSA, 0))
        {
            return false;
        }
        FDCAN3->CCCR |= FDCAN_CCCR_INIT;
        if (!board::waitBits(FDCAN3->CCCR, FDCAN_CCCR_INIT, FDCAN_CCCR_INIT))
        {
            return false;
        }
        FDCAN3->CCCR |= FDCAN_CCCR_CCE;
        FDCAN_CONFIG->CKDIV = 0;
        /// 24 MHz / (1 * (1 + 18 + 5)) = 1 Mbps, sample point 79.17%, SJW 4 tq.
        FDCAN3->NBTP = (3UL << FDCAN_NBTP_NSJW_Pos) | (17UL << FDCAN_NBTP_NTSEG1_Pos) | (4UL << FDCAN_NBTP_NTSEG2_Pos);
        FDCAN3->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_TEST | FDCAN_CCCR_MON);
        /// Accept data frames into FIFO0; reject remote frames. Protocol filters come later.
        FDCAN3->RXGFC = FDCAN_RXGFC_RRFE | FDCAN_RXGFC_RRFS;
        FDCAN3->TXBC = 0;  ///< FIFO mode.
        auto* ram = reinterpret_cast<volatile uint32_t*>(SRAMCAN_BASE + 2 * InstanceBytes);
        for (unsigned i = 0; i < InstanceBytes / 4; ++i)
        {
            ram[i] = 0;
        }
        FDCAN3->IR = 0xFFFFFFFF;
        FDCAN3->IE = FDCAN_IE_RF0NE | FDCAN_IE_RF0LE | FDCAN_IE_BOE;
        FDCAN3->ILS = 0;
        FDCAN3->ILE = FDCAN_ILE_EINT0;
        NVIC_SetPriority(FDCAN3_IT0_IRQn, 8);
        NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
        /// Switch to AF only after the controller drives the recessive idle state.
        board::pinMode(GPIOB, 3, 2, 11);
        board::pinMode(GPIOB, 4, 2, 11);
        FDCAN3->CCCR &= ~FDCAN_CCCR_INIT;
        _ready = board::waitBits(FDCAN3->CCCR, FDCAN_CCCR_INIT, 0);
        return _ready;
    }

    /**
     * @brief 将数据帧提交至硬件 TX FIFO，不等待总线发送完成。
     * @param frame 待发送的数据帧。
     * @return 成功提交返回 true；未就绪、bus-off、参数非法或 FIFO 满时返回 false。
     */
    bool can::send(const can_frame& frame)
    {
        if (!_ready || _bus_off || frame.size > 8 || frame.id > (frame.extended ? 0x1FFFFFFFUL : 0x7FFUL))
        {
            return false;
        }
        board::critical_section lock;
        if ((FDCAN3->TXFQS & FDCAN_TXFQS_TFQF) != 0)
        {
            return false;
        }
        const unsigned index = (FDCAN3->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
        auto* ram = element(TxOffset, index);
        ram[0] = frame.extended ? frame.id | (1UL << 30) : frame.id << 18;
        ram[1] = static_cast<uint32_t>(frame.size) << 16;
        for (unsigned word = 0; word < 2; ++word)
        {
            uint32_t value = 0;
            for (unsigned byte = 0; byte < 4; ++byte)
            {
                const unsigned i = word * 4 + byte;
                if (i < frame.size)
                {
                    value |= static_cast<uint32_t>(frame.data[i]) << (byte * 8);
                }
            }
            ram[2 + word] = value;
        }
        __DMB();
        FDCAN3->TXBAR = 1UL << index;
        return true;
    }

    /// @brief 处理接收、帧丢失和 bus-off 中断，单次最多读取三帧。
    void can::handleIrq()
    {
        const uint32_t flags = FDCAN3->IR;
        FDCAN3->IR = flags;
        if ((flags & FDCAN_IR_RF0L) != 0)
        {
            ++_errors;
        }
        if ((flags & FDCAN_IR_BO) != 0 && (FDCAN3->PSR & FDCAN_PSR_BO) != 0)
        {
            _bus_off = true;
            ++_errors;  ///< Recovery must be requested by a future supervisory module.
        }
        for (unsigned received = 0; received < 3; ++received)
        {
            const uint32_t status = FDCAN3->RXF0S;
            if ((status & FDCAN_RXF0S_F0FL) == 0)
            {
                break;
            }
            const unsigned index = (status & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
            auto* ram = element(RxOffset, index);
            const uint32_t header = ram[0];
            const uint32_t control = ram[1];
            const unsigned next = (_head + 1) % 16;
            const unsigned size = (control >> 16) & 15;
            if (next == _tail || size > 8 || (control & (1UL << 21)) != 0 || (header & (1UL << 29)) != 0)
            {
                ++_errors;
            }
            else
            {
                auto& frame = _rx[_head];
                frame.extended = (header & (1UL << 30)) != 0;
                frame.id = frame.extended ? header & 0x1FFFFFFF : (header >> 18) & 0x7FF;
                frame.size = static_cast<uint8_t>(size);
                for (unsigned i = 0; i < 8; ++i)
                {
                    frame.data[i] = static_cast<uint8_t>(ram[2 + i / 4] >> ((i % 4) * 8));
                }
                __DMB();
                _head = next;
            }
            FDCAN3->RXF0A = index;
        }
        if ((FDCAN3->RXF0S & FDCAN_RXF0S_F0FL) != 0)
        {
            NVIC_SetPendingIRQ(FDCAN3_IT0_IRQn);
        }
    }

    /**
     * @brief 从软件接收队列取出一帧。
     * @param[out] frame 成功时写入接收帧。
     * @return 成功取出返回 true，队列为空返回 false。
     */
    bool can::receive(can_frame& frame)
    {
        board::critical_section lock;
        if (_tail == _head)
        {
            return false;
        }
        frame = _rx[_tail];
        _tail = (_tail + 1) % 16;
        return true;
    }
}
/**
 * @brief 转发 FDCAN3 接收与状态中断。
 */
extern "C" void FDCAN3_IT0_IRQHandler()
{
    drivers::can::instance().handleIrq();
}
