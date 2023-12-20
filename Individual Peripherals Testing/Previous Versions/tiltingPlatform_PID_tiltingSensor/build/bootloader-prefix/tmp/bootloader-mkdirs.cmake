# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.0/components/bootloader/subproject"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix/tmp"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix/src"
  "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Personal/SeniorCapstone/tiltingPlatform_PID_tiltingSensor/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
