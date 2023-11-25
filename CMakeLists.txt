cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(candle)

pico_sdk_init()


add_executable(candle
	candle.c
	)

target_include_directories(candle PRIVATE .)

target_link_libraries(candle
	pico_stdlib
	pico_multicore
	hardware_pwm
	hardware_adc
)

pico_add_extra_outputs(candle)

