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

## Attribution

Content from the following external sources is used within this project:

* [wpilibsuite/allwpilib](https://github.com/wpilibsuite/allwpilib/tree/v2021.3.1)
* [CrossTheRoadElec/Phoenis-Linux-SocketCAN-Example](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example/tree/9ed6d7f370c6effea245790cd5c5d393a3f26a6a)
* [FRC1756-Argos/ArgosLib-Cpp](https://github.com/FRC1756-Argos)
* [tttapa/RPi-CPP-Toolchain](https://github.com/tttapa/RPi-Cpp-Toolchain/tree/103eb26a24a7e03b9672638ece72881311a6c9df)
* [SDL2](https://www.libsdl.org/download-2.0.php)
