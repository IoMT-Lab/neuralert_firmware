cmake_minimum_required(VERSION 3.20)

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
  set(TOOLCHAIN_PATH ${CMAKE_SOURCE_DIR}/arm-toolchain/linux-x86_64/gcc-arm-none-eabi-10.3-2021.10)
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
  set(TOOLCHAIN_PATH ${CMAKE_SOURCE_DIR}/arm-toolchain/mac-x86_64/gcc-arm-none-eabi-10.3-2021.10)
else()

endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/da16200_sdk/cmake)
set(CMAKE_TOOLCHAIN_FILE da16200_sdk/cmake/da16200-toolchain.cmake)

project(neuralert C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

add_subdirectory(neuralert)

set(MAIN_TARGET ${NEURALERT_TARGET})
set(MAIN_TARGET_NAME ${NEURALERT_TARGET_NAME})
set(MAIN_TARGET_INCLUDE ${NEURALERT_TARGET_INCLUDE})

add_subdirectory(da16200_sdk)
target_link_libraries(${MAIN_TARGET} DA16200_SDK_MAIN)

