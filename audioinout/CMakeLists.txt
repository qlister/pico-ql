# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17) 

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/qlister/pico/pico-sdk")

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(audioinout C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

pico_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/audio_i2s)
#pico_add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/common)

add_executable(audioinout audioinout.cpp )

pico_set_program_name(audioinout "audioinout")
pico_set_program_version(audioinout "0.1")

pico_enable_stdio_uart(audioinout 1)
pico_enable_stdio_usb(audioinout 0)

# Add the standard library to the build
target_link_libraries(audioinout
        audio_i2s_handler
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(audioinout PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)


target_compile_definitions(audioinout PRIVATE
# compile time configuration of I2S
#PICO_AUDIO_I2S_MONO_INPUT=1
#define for our example code
#USE_AUDIO_I2S=1
PICO_AUDIO_I2S_IN_DATA_PIN=28
PICO_AUDIO_I2S_IN_CLOCK_PIN_BASE=26

PICO_AUDIO_I2S_OUT_DATA_PIN=22
PICO_AUDIO_I2S_OUT_CLOCK_PIN_BASE=20

#            PICO_DEFAULT_UART=0
#            PICO_DEFAULT_UART_TX_PIN=28
#            PICO_DEFAULT_UART_RX_PIN=29
)

# Add any user requested libraries
target_link_libraries(audioinout
        hardware_pio
        pico_stdlib
        )

pico_add_extra_outputs(audioinout)


