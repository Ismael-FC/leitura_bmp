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

project(leitura_bmp C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(leitura_bmp leitura_bmp.c )

pico_set_program_name(leitura_bmp "leitura_bmp")
pico_set_program_version(leitura_bmp "0.1")

# Generate PIO header
pico_generate_pio_header(leitura_bmp ${CMAKE_CURRENT_LIST_DIR}/leitura_bmp.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(leitura_bmp 0)
pico_enable_stdio_usb(leitura_bmp 1)

# Add the standard library to the build
target_link_libraries(leitura_bmp
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(leitura_bmp PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(leitura_bmp 
        hardware_pio
        hardware_uart
        )

# target_compile_definitions(leitura_bmp PRIVATE
#         PICO_DEFAULT_UART_TX_PIN=4
#         PICO_DEFAULT_UART_RX_PIN=5
#         PICO_DEFAULT_UART=1
# )

pico_add_extra_outputs(leitura_bmp)


