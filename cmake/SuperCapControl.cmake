set(board_root "${PROJECT_SOURCE_DIR}/boards/supercap")
set(third_party_root "${PROJECT_SOURCE_DIR}/ThirdParty")
set(hal_root "${third_party_root}/ST/STM32G4xx_HAL_Driver")
set(cmsis_root "${third_party_root}/ST/CMSIS")
set(linker_script "${board_root}/stm32g474cb_flash.ld")

add_library(mcu_options INTERFACE)
target_compile_options(mcu_options INTERFACE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -O2 -g -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics>)
target_link_options(mcu_options INTERFACE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -T${linker_script} --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_BINARY_DIR}/supercap.map)

set(hal_modules hal hal_adc hal_adc_ex hal_cortex hal_dma hal_dma_ex hal_fdcan
    hal_flash hal_flash_ex hal_gpio hal_hrtim hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex
    hal_uart hal_uart_ex ll_adc)
list(TRANSFORM hal_modules PREPEND "${hal_root}/Src/stm32g4xx_")
list(TRANSFORM hal_modules APPEND ".c")
add_library(stm32_hal STATIC ${hal_modules} "${board_root}/system_stm32g4xx.c")
target_include_directories(stm32_hal SYSTEM PUBLIC
    "${hal_root}/Inc"
    "${cmsis_root}/Device/ST/STM32G4xx/Include"
    "${cmsis_root}/Include")
target_include_directories(stm32_hal PUBLIC "${board_root}")
target_compile_definitions(stm32_hal PUBLIC STM32G474xx USE_HAL_DRIVER HSE_VALUE=24000000U)
target_link_libraries(stm32_hal PRIVATE mcu_options)
