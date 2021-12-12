# Swerve-Platform

Robotic swerve drive platform developed for Northwoods Church Christmas program

## How To Build

### Prerequisites

Tested in Linux, but might work in other environments because most of build is conducted in Docker for cross compilation.

* [Docker](https://docs.docker.com/get-docker/)
* [CMake](https://cmake.org/install/)

### Build Steps

1. `mkdir build`
2. `cd build`
3. `../utils/toolchain.sh rpi3-aarch64 --pull --export`
4. `cmake -DCMAKE_TOOLCHAIN_FILE=../utils/aarch64-rpi3-linux-gnu.cmake -DCMAKE_BUILD_TYPE=Debug ..`
5. ``make -j`nproc` ``

Steps 1-4 may be skipped once the environment is set up appropriately.

## SSH
pi@NWCC-platform-alpha

## Required software packages

### Linux platform software
'sudo apt-get update'
'sudo apt-get upgrade'

### hostname loopback
Add the following line to '/etc/hosts'
'127.0.1.1 NWCC-platform-alpha'

### PiCAN
Add the following lines to '/boot/config.txt'
'dtparam=spi=on'
'dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25'
'dtoverlay=spi-bcm2835-overlay'

### CAN-Utils
'sudo apt-get install can-utils'

## Attribution

Content from the following external sources is used within this project:

* [wpilibsuite/allwpilib](https://github.com/wpilibsuite/allwpilib/tree/v2021.3.1)
* [CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example/tree/9ed6d7f370c6effea245790cd5c5d393a3f26a6a)
* [FRC1756-Argos/ArgosLib-Cpp](https://github.com/FRC1756-Argos)
* [tttapa/RPi-CPP-Toolchain](https://github.com/tttapa/RPi-Cpp-Toolchain/tree/103eb26a24a7e03b9672638ece72881311a6c9df)
* [SDL2](https://www.libsdl.org/download-2.0.php)
