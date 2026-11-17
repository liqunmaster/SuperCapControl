cmake_minimum_required(VERSION 3.21)
project(SuperCapControl LANGUAGES C CXX ASM)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(SUPERCAP_CAN_WIRING_CONFIRMED "PB3 RX / PB4 TX physically verified" OFF)
include(cmake/SuperCapControl.cmake)
add_executable(supercap
    boards/startup_stm32g474rbtx.s
    boards/supercap/board.cpp
    drivers/uart_dma.cpp
    drivers/adc_dma.cpp
    drivers/can.cpp
    drivers/hrtim.cpp
    src/main.cpp
    src/syscalls.cpp)
target_include_directories(supercap PRIVATE boards/supercap drivers)
target_include_directories(supercap SYSTEM PRIVATE boards/CMSIS)
target_compile_definitions(supercap PRIVATE STM32G474xx
    SUPERCAP_CAN_WIRING_CONFIRMED=$<BOOL:${SUPERCAP_CAN_WIRING_CONFIRMED}>)
target_link_libraries(supercap PRIVATE mcu_options)
set_target_properties(supercap PROPERTIES SUFFIX .elf)
set_property(TARGET supercap APPEND PROPERTY LINK_DEPENDS
    ${PROJECT_SOURCE_DIR}/boards/supercap/stm32g474cb_flash.ld)
add_custom_command(TARGET supercap POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:supercap> ${CMAKE_CURRENT_BINARY_DIR}/supercap.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:supercap> ${CMAKE_CURRENT_BINARY_DIR}/supercap.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:supercap>)
find_program(OPENOCD openocd)
if(OPENOCD)
    add_custom_target(flash
        COMMAND ${OPENOCD} -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg
            -c "program $<TARGET_FILE:supercap> verify reset exit"
        DEPENDS supercap VERBATIM)
endif()
