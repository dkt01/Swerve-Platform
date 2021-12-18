set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7)

set(TOOLCHAIN_BIN "${CMAKE_CURRENT_LIST_DIR}/x-tools/armv8-rpi3-linux-gnueabihf/bin")
set(CMAKE_C_COMPILER "${TOOLCHAIN_BIN}/armv8-rpi3-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN}/armv8-rpi3-linux-gnueabihf-g++")

set(CMAKE_SYSROOT "${CMAKE_CURRENT_LIST_DIR}/sysroot-armv8-rpi3-linux-gnueabihf")
SET(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")

include("${CMAKE_CURRENT_LIST_DIR}/Common-RPi-Toolchain.cmake")
