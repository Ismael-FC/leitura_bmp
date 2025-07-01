# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(bmp_cycle C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(bmp_cycle ${CMAKE_CURRENT_LIST_DIR}/src/bmp_cycle.c )

pico_set_program_name(bmp_cycle "bmp_cycle")
pico_set_program_version(bmp_cycle "0.1")

# Generate PIO header
pico_generate_pio_header(bmp_cycle ${CMAKE_CURRENT_LIST_DIR}/src/bmp_cycle.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(bmp_cycle 0)
pico_enable_stdio_usb(bmp_cycle 1)

# Add the standard library to the build
target_link_libraries(bmp_cycle
        pico_stdlib
        pico_multicore
        )

# Add the standard include files to the build
target_include_directories(bmp_cycle PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        include
)

# Add any user requested libraries
target_link_libraries(bmp_cycle 
        hardware_pio
        hardware_uart
        )

# Prints memory usage after compiling
target_link_options(bmp_cycle PRIVATE -Wl,--print-memory-usage)

# O2 and O3 compile, but the program does not run correctly
target_compile_options(bmp_cycle PRIVATE -Wall -O1)

pico_add_extra_outputs(bmp_cycle)


