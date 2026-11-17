# 板级配置与验收

## 输入依据与边界

芯片：用户确认的 STM32G474CBT6；外部晶振：截图标注 24 MHz。
仅从 `C:/Users/LiQun/Desktop/CAP_Crl/cap.ioc` 读取芯片标识与引脚字段，用于核对；
没有照搬旧工程的时钟、算法、DMA、控制周期、驱动代码。

依据：

- [ST DS12288 数据手册](https://www.st.com/resource/en/datasheet/stm32g474cb.pdf)：封装、存储器、ADC 通道与 AF 表。
- [ST RM0440 参考手册](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)：寄存器、DMA 请求、外设时序；实施时结合工程 CMSIS 定义和 ST 官方驱动定义核查。
- [ST 官方 DMA 请求定义](https://github.com/STMicroelectronics/stm32g4xx-hal-driver/blob/master/Inc/stm32g4xx_hal_dma.h)。
- [ST 官方 FDCAN RAM 布局](https://github.com/STMicroelectronics/stm32g4xx-hal-driver/blob/master/Src/stm32g4xx_hal_fdcan.c)：G4 固定布局与 72 字节元素步长。
- [PX4 架构](https://docs.px4.io/main/en/concept/architecture)、[PX4 风格](https://docs.px4.io/main/en/contribute/code)。

## 引脚

| 信号 | MCU 引脚 | 外设/通道 |
| --- | --- | --- |
| Capacitor_Voltage | PA5 | ADC2 IN13 |
| Capacitor_Current | PA6 | ADC2 IN3 |
| Chassis_Voltage | PA7 | ADC2 IN4 |
| Chassis_Current | PB0 | ADC1 IN15 |
| Output_Voltage | PB1 | ADC1 IN12 |
| Output_Current | PB2 | ADC2 IN12 |
| CHA1 / CHA2 | PA8 / PA9 | HRTIM1 AF13，当前保持 GPIO 输出低 |
| CHB1 / CHB2 | PA10 / PA11 | HRTIM1 AF13，当前保持 GPIO 输出低 |
| CAN RX / TX | PB3 / PB4 | FDCAN3 AF11，默认不切换 AF、不启动 |
| DEBUG TX / RX | PB10 / PB11 | USART3 AF7 |
| LED G / B / R / P | PB12 / PB13 / PB14 / PB15 | GPIO，高电平亮 |
| SWDIO / SWCLK | PA13 / PA14 | 保持调试功能 |
| HSE | PF0 / PF1 | 24 MHz 晶振 |

截图把 PB3 标成 CAN TX、PB4 标成 CAN RX，与芯片固定复用相反；旧工程 IOC 选择的是正确复用。
必须实际确认 PB4 接收发器 TXD、PB3 接 RXD 后才能启用：

```powershell
cmake --preset debug -DSUPERCAP_CAN_WIRING_CONFIRMED=ON
cmake --build --preset debug
```

此选项在 CMake 缓存中持久保存；恢复禁用时显式传入 `OFF`。

## 时钟与数据路径

HSE 24 MHz → PLL M=6/N=72/R=2 → SYSCLK/AHB/APB1/APB2 144 MHz；
电压范围 1 普通模式，Flash 4 等待周期，中间 AHB /2 后切换全速。
HSE/PLL/ADC/UART 就绪均有有界等待，失败进入 `panic`，保持功率输出低。
超时为循环次数上限，不宣称精确毫秒超时。

### ADC

TIM6 TRGO 1 kHz 同时触发 ADC1/ADC2 常规序列，ADC 时钟 HCLK/4 = 36 MHz，采样时间 47.5 周期，12 位单端。
ADC1 扫描 IN15/IN12，ADC2 扫描 IN13/IN3/IN4/IN12。
两组独立 DMA 循环缓冲，每半缓冲为一整帧；HT/TC 中断复制已完成半区，前台仅取得两组序号匹配的数据。
各通道依序转换，不是六路同时采样；禁止把当前序列解释为已验证的同步电流环采样。
DMA 错误、HT/TC 同时积压或 ADC overrun 会停止 TIM6/DMA，关闭 HRTIM 输出并记录错误，不自动重启。

输出仅为原始 ADC 码。外部参考电压、分压比、运放增益/偏置和传感器极性尚未标定，不输出臆测的伏特/安培值。
1 kHz、47.5 周期是本阶段采样验证设置，不是最终控制参数。
后续确定开关周期后改为 HRTIM 硬件触发，校核采样窗口、RC 建立时间、通道顺序和闭环延迟。

### UART / printf

USART3 115200 8N1；TX 使用 DMA1 CH1、DMAMUX 请求 29、1024 字节有界环形队列。
`_write` 连接 newlib `printf`。仅允许前台调用，DMA ISR 只推进队列，不格式化、不等待。
每次临界区最多复制 64 字节，实际屏蔽中断时长仍需上板测量。
队列满时短写，无空位返回 `-1/EAGAIN`；调用方如使用 stdio，应检查返回值并按需 `clearerr`，不无限重试。
DMA 错误丢弃当前块、累计错误，再继续下一块；日志可能丢失，不将日志作为控制数据链。
RX 当前采用 RXNE/错误中断和 128 字节队列，满队列丢新字节；尚未实现 RX DMA/IDLE。
禁止在控制中断、关闭中断区或未来 RTOS 多任务中直接并发调用 `printf`；未来应设置单独日志任务/生产者接口。
驱动无动态分配；newlib stdio 预留有界 2 KiB 堆，保留至少 4 KiB 主栈。

### CAN

FDCAN3 经典 CAN，关闭 FD/BRS；HSE 24 MHz，分频 1，1+18+5=24 tq，1 Mbps，采样点 79.17%，SJW 4 tq。
支持标准/扩展数据帧，0..8 字节；接收中断进入 16 项环形队列（有效 15 项），发送使用硬件 FIFO。
G4 FDCAN 不使用通用 DMA，按外设固定消息 SRAM 访问。
当前接收所有数据 ID、拒绝远程帧；协议 ID、过滤规则、应用超时和 bus-off 恢复策略待确认。
软件队列满丢新帧并计数；bus-off 锁存状态并拒绝发送，不自动恢复，不在 ISR 里操作业务。

### HRTIM

保持 ODISR 禁用输出、PA8..11 为推挽低电平、所有计数器停止，没有启用输出 API。
`prepareCompareDma` 将每路至多 64 个比较值复制到驱动拥有的 SRAM，配置 DMA1 CH4/CH5、请求 96/97、CMP1 目标。
**此接口只完成准备，不开启 DMA EN、定时器 DMA 请求、DLL、计数器、复用输出或互补波形。**
比较值相对周期的范围、A/B 同步生效、死区和首周期更新均尚未验证，不能当作可运行的 PWM 驱动。
后续需要开关频率、死区、最大占空比、双向工作策略、采样相位、故障输入和关断要求，
再实现预装载/更新事件及 DMA 启停状态机。单个比较值 DMA 不等价于两桥臂参数原子更新。

## 中断预算

数值小者优先：HRTIM DMA 1（仅预留）、ADC overrun 2、ADC DMA 3、FDCAN 8、UART TX DMA 10、UART RX 11、SysTick 15。
当前 1 ms tick 只用于启动演示和日志，控制算法没有放入 SysTick。
未来 RTOS 的系统调用优先级门槛需要重新审查；高优先级控制 ISR 不调用 RTOS/stdio API。

## 验收与待办

- [x] ARM GCC Debug/Release 构建，生成 ELF/HEX/BIN/MAP 与编译数据库。
- [x] 七个新增 C++ 文件的 clangd 解析检查（关闭 clangd 自身重构动作试验）。
- [x] 独立 CAN 启用构建通过；日常 Debug/Release 仍保持 CAN 禁用。
- [x] ELF 中断符号、UART/ADC 缓冲 SRAM 地址和可加载段权限检查。
- [ ] DAPLink 连接、SWD 复位与烧录验证。
- [ ] 示波器确认启动、复位、异常期间 PA8..11 保持低；实测 SYSCLK 和晶振。
- [ ] UART 连续输出、发送队列满、DMA 故障和 RX 溢出验证。
- [ ] 六路已知输入 ADC 原始码核对，校准参考电压及电流零点，确认无 overrun。
- [ ] CAN 物理 TX/RX 核对，双节点 1 Mbps 标准/扩展帧收发、队列满和 bus-off 验证。
- [ ] 在功率级隔离条件下确定 PWM/ADC 参数，验证 DMA 更新、互补死区、硬件关断路径和中断最坏延迟。
- [ ] 确认控制目标及约束后设计并优化算法，再决定是否需要 RTOS。

构建与静态检查不能验证电气连接、采样精度、功率波形或实时性。当前没有进行烧录或功率级实验。
