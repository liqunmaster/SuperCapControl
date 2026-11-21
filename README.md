# SuperCapControl

STM32G474CBT6 超级电容固件。使用 C++14、STM32G4 HAL、ARM GCC、CMake、Ninja 和 clangd。本阶段提供 ADC、PWM、GPIO、CAN 和 UART 驱动，应用入口用于验证驱动和调度。

## 构建与调试

要求 ARM GNU Toolchain（GCC 11+）、CMake 3.21+ 和 Ninja。工具可放在 PATH 中，或在未纳入版本控制的 `cmake/tools.local.cmake` 中配置，例如：

```cmake
set(SUPERCAP_TOOL_PATHS
    "D:/MCU/flash/GCC/bin"
    "D:/MCU/flash/Ninja"
    "D:/MCU/flash/OpenOCD/bin")
```

```powershell
cmake --preset firmware
cmake --build --preset firmware
```

固件只有 `firmware` 一个预设，使用 `-O2 -g`。产物在 `build`，包括 `supercap.elf`、`supercap.hex`、`supercap.bin`、`supercap.map` 和 `compile_commands.json`。VS Code 的 `build` 任务使用同一个预设，可在 `DAPLink SWD` 配置下调试。

功率级隔离、仅接逻辑电源时，可通过 OpenOCD 和 DAPLink 烧录：

```powershell
cmake --build --preset firmware --target flash
```

## 工程分层

- `boards/supercap` 管理板级时钟、引脚默认状态、芯片启动和系统初始化文件、链接布局以及 HAL 的 TIM2 时间基准。
- `drivers` 封装外设及 DMA、中断交接。应用通过公开接口读取采样、接收消息和提交驱动操作。
- `application` 管理 31.25 kHz 控制中断、采样窗口消费和前台通信。`src` 保留启动入口与 newlib 标准流重定向。
- `ThirdParty/ST` 提供 HAL 和 CMSIS。裸机固件未链接 FreeRTOS。
- `math` 保存已有数值算法，供后续观测器和控制模块接入。

目录和职责沿用类似 PX4 的板级、驱动、应用分层。自有类型使用小写，方法使用 `lowerCamelCase`，私有字段带下划线前缀。项目开发约定见 [AGENTS.md](AGENTS.md)。

[板级配置、中断预算和上板验收](docs/bringup.md)。
