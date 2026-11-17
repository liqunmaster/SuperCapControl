add_library(mcu_options INTERFACE)
target_compile_options(mcu_options INTERFACE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -ffunction-sections -fdata-sections
    $<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-Wextra;-Wshadow;-Wconversion>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics>
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-O2;-g>)
target_link_options(mcu_options INTERFACE
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
    -T${PROJECT_SOURCE_DIR}/boards/supercap/stm32g474cb_flash.ld
    --specs=nano.specs --specs=nosys.specs
    -Wl,--gc-sections -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_BINARY_DIR}/supercap.map)
