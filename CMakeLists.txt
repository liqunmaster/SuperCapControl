cmake_minimum_required(VERSION 3.21)
project(SuperCapControl LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(SUPERCAP_CAN_WIRING_CONFIRMED "PB3 RX / PB4 TX physically verified" OFF)

include(cmake/SuperCapControl.cmake)

add_executable(supercap
    "${board_root}/startup_stm32g474cbtx.s"
    boards/supercap/board.cpp
    boards/supercap/timebase.cpp
    application/control.cpp
    math/incremental_pid.cpp
    drivers/gpio.cpp
    drivers/adc.cpp
    drivers/hrtim.cpp
    drivers/can.cpp
    drivers/uart.cpp
    src/main.cpp
    src/syscalls.cpp)
set(config_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(GENERATE OUTPUT "${config_dir}/board_options.hpp"
    CONTENT "#pragma once\nnamespace board { constexpr bool can_wiring_confirmed = $<IF:$<BOOL:${SUPERCAP_CAN_WIRING_CONFIRMED}>,true,false>; }\n")
target_include_directories(supercap PRIVATE boards/supercap drivers application math "${config_dir}")
target_compile_options(supercap PRIVATE -Wall -Wextra -Wshadow -Wconversion)
target_link_libraries(supercap PRIVATE stm32_hal mcu_options)
set_target_properties(supercap PROPERTIES SUFFIX .elf)
set_property(TARGET supercap APPEND PROPERTY LINK_DEPENDS "${linker_script}")

add_custom_command(TARGET supercap POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:supercap> ${CMAKE_CURRENT_BINARY_DIR}/supercap.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:supercap> ${CMAKE_CURRENT_BINARY_DIR}/supercap.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:supercap>
    VERBATIM)
find_program(OPENOCD openocd)
if(OPENOCD)
    add_custom_target(flash
        COMMAND ${OPENOCD} -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg
            -c "program $<TARGET_FILE:supercap> verify reset exit"
        DEPENDS supercap VERBATIM)
endif()
