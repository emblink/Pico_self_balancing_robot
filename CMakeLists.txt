# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_PROJECT_NAME Self_balancing_robot)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# === Make FreeRTOS headers available early (important to avoid implicit-declaration errors) ===
# Paths relative to the project root
set(FREERTOS_DIR "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel")
set(FREERTOS_PORT_DIR "${FREERTOS_DIR}/portable/ThirdParty/GCC/RP2040")

# Add FreeRTOS includes globally (BEFORE pico headers are processed)
include_directories(BEFORE
    "${FREERTOS_DIR}/include"
    "${FREERTOS_PORT_DIR}/include"
)
# ============================================================================================

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include("${FREERTOS_PORT_DIR}/FreeRTOS_Kernel_import.cmake")

project(${CMAKE_PROJECT_NAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${CMAKE_PROJECT_NAME}
    src/main.c
)

pico_set_program_name(${CMAKE_PROJECT_NAME} "${CMAKE_PROJECT_NAME}")
pico_set_program_version(${CMAKE_PROJECT_NAME} "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)

# Add the standard library to the build
target_link_libraries(${CMAKE_PROJECT_NAME}
    pico_stdlib
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    )

# Add the standard include files to the build
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        include/
        "${FREERTOS_DIR}/include"
        "${FREERTOS_PORT_DIR}/include"
)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
