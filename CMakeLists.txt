cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS_RELEASE "-Ofast -ffast-math -frename-registers")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")


pico_sdk_init()

add_executable(pico_hacksaw
  pico_hacksaw.c
)

pico_set_binary_type(pico_hacksaw copy_to_ram)

pico_generate_pio_header(pico_hacksaw ${CMAKE_CURRENT_LIST_DIR}/dsp_clock.pio )


pico_enable_stdio_usb(pico_hacksaw 1)
pico_enable_stdio_uart(pico_hacksaw 0)
pico_add_extra_outputs(pico_hacksaw)

target_link_libraries(pico_hacksaw pico_stdlib hardware_pio hardware_adc hardware_dma pico_multicore)