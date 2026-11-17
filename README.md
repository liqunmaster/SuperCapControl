# SuperCapControl

STM32G474CBT6 超级电容控制工程，C++14、CMake、Ninja、CMSIS 寄存器映射，无 HAL、无 RTOS。
当前交付范围是构建环境、板级初始化和基础驱动；尚未实现功率闭环算法。

## 构建与调试

要求 ARM GNU Toolchain（GCC 11+）、CMake 3.21+、Ninja；调试使用 OpenOCD、DAPLink/CMSIS-DAP、SWD。
工具可以在 PATH 中，也可以在不提交版本控制的 `cmake/tools.local.cmake` 中配置：

```cmake
set(SUPERCAP_TOOL_PATHS
    "D:/MCU/flash/GCC/bin"
    "D:/MCU/flash/Ninja"
    "D:/MCU/flash/OpenOCD/bin")
```

当前机器已配置该文件。当前机器的 clangd GCC 查询路径配置在 `.vscode/settings.json`，换机器时修改该项。

```powershell
cmake --preset debug
cmake --build build
cmake --preset release
cmake --build --preset release
```

默认构建类型为 Debug，构建目录为 `build`。首次配置运行 `cmake --preset debug`，之后直接运行 `cmake --build build`。
Release 保留独立目录 `build/release`。

产物：`build/supercap.elf`、`.hex`、`.bin`、`.map`、`compile_commands.json`。
VS Code 安装推荐扩展后，Ctrl+Shift+B 构建，选择 `DAPLink SWD` 调试。
如工具不在 PATH，通过本地环境或 CMake 用户预设指定工具路径，不提交个人绝对路径到工具链。

烧录会操作真实硬件，应先确认功率级断电、仅逻辑电源供电：

```powershell
cmake --build --preset debug --target flash
```

## 架构

- `boards/supercap`：板级时钟、引脚工具、启动支持、内存布局。
- `drivers`：UART DMA、ADC DMA、经典 CAN、HRTIM 基础接口；类封装状态，通过公开 IRQ 接口转发中断。
- `src`：裸机入口、newlib 系统调用；应用决定何时读取数据，不在驱动中写控制业务。
- `math`：保留已有数学库。本阶段固件未依赖它，未修改现有算法；其原有独立测试配置不属于本次构建。

借鉴 PX4 的板级、驱动、应用职责划分及 `lowerCamelCase` 方法、私有成员前导下划线；
按本项目要求使用 4 空格，而非 PX4 的 Tab。不引入 PX4/NuttX、uORB 或工作队列框架。

自有类型名称全部使用小写，例如 `drivers::uart_dma`、`drivers::adc_dma`、`drivers::hrtim`。
矩阵类型已统一为 `matrix::fixed_matrix`，原 `Matrix` 类型名称不再保留。
命名空间、访问控制符及成员按 `.clang-format` 逐级缩进；公开接口使用 Doxygen 注释，函数说明优先写在 `.cpp` 的定义处。仅在头文件定义的接口按需在 `.hpp` 中记录。

启动向量复用已有 `startup_stm32g474rbtx.s`：该文件是 G474 系列向量实现，RB 文件名不决定封装和容量。
实际使用新建的 CB 链接脚本：128 KiB Flash、96 KiB 常规 SRAM、32 KiB 独立 CCM。
DMA 缓冲全部位于常规 SRAM；原 RB 链接脚本不参与构建。

引脚、驱动限制、验收步骤见 [bringup.md](docs/bringup.md)。
