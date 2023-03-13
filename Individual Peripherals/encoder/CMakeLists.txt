
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# (Not part of the boilerplate)
# This example uses an extra component for common functions such as Wi-Fi and Ethernet connection.
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
set(I2C C:/Personal/SeniorCapstone/accel_New/components/i2c_lib)
set(acc C:/Personal/SeniorCapstone/accel_New/components/accel_reg)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(encoder)
