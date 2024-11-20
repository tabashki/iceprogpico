# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(iceprogpico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(iceprogpico iceprogpico.c )

pico_set_program_name(iceprogpico "iceprogpico")
pico_set_program_version(iceprogpico "0.1")

# Only use USB for serial communication
pico_enable_stdio_usb(iceprogpico 1)
pico_enable_stdio_uart(iceprogpico 0)

# Add the standard library to the build
target_link_libraries(iceprogpico pico_stdlib)

# Add the standard include files to the build
target_include_directories(iceprogpico PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(iceprogpico hardware_spi)

# Doesn't make sense to run this program from flash
pico_set_binary_type(iceprogpico no_flash)

pico_add_extra_outputs(iceprogpico)
