cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(mpu9250_i2c C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# Micro-ROS
link_directories($ENV{HOME}/pico/micro_ros_raspberrypi_pico_sdk/libmicroros)

add_compile_options(-Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  -Wno-maybe-uninitialized
  )

add_executable(mpu9250_i2c
  main.c
  mpu9250_i2c.c
  uart/pico_uart_transport.c
  )

target_include_directories(mpu9250_i2c PUBLIC
  $ENV{HOME}/pico/micro_ros_raspberrypi_pico_sdk/libmicroros/include
  uart
)

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(mpu9250_i2c
  pico_stdlib
  hardware_i2c
  pico_multicore
  microros
  )

# Configure Pico
pico_enable_stdio_usb(mpu9250_i2c 1)
pico_enable_stdio_uart(mpu9250_i2c 1)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu9250_i2c)
