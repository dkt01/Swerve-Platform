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
3. `../utils/toolchain.sh rpi3-armv8 --pull --export`
4. `cmake -DCMAKE_TOOLCHAIN_FILE=../utils/armv8-rpi3-linux-gnueabihf.cmake -DCMAKE_BUILD_TYPE=Debug ..`
5. ``make -j`nproc` ``

Steps 1-4 may be skipped once the environment is set up appropriately.

## SSH
`ssh pi@NWCC-platform-alpha.local`

## How To Run At Startup

1. Copy `bin`, `lib`, and `scripts` to `/home/pi/swerve-platform/`
2. Copy `scripts/swerve-platform.service` to `/etc/systemd/system/`: `sudo cp /home/pi/swerve-platform/scripts/swerve-platform.service /etc/systemd/system`
3. Adjust permissions of unit file: `sudo chmod 644 /etc/systemd/system/swerve-platform.service`
4. Enable service: `sudo systemctl enable swerve-platform.service`
5. Reboot

## Required Software Packages

### Linux Platform Software

1. `sudo apt-get update`
2. `sudo apt-get upgrade`

### hostname Loopback

Add the following lines to `/etc/hosts`
1. `127.0.1.1 NWCC-platform-alpha`

### PiCAN

Add the following lines to `/boot/config.txt`
1. `dtparam=spi=on`
2. `dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25`
3. `dtoverlay=spi-bcm2835-overlay`

### CAN-Utils

1. `sudo apt-get install can-utils`

### SDL2

1. `sudo apt-get install libsdl2-dev`

## Bare Minimum To Pair XBox Series Controller

1. Update controller firmware in Windows using [XBox Accessories App](https://www.microsoft.com/en-us/p/xbox-accessories/9nblggh30xj3)
2. `echo 'options bluetooth disable_ertm=Y' | sudo tee -a /etc/modprobe.d/bluetooth.conf`
3. `systemctl edit bthelper@*` and add the following content (exact name of this service may change after the `@`):
   ```
   [Unit]
   After=hciuart.service bluetooth.service
   Before=

   [Service]
   ExecStartPre=/bin/sleep 5
   ```
4. Add `Privacy = device` to /etc/bluetooth/main.conf
5. Reboot
6. Run the following with XBox controller in pair mode.  Lines after `sudo bluetoothctl` are run inside bluetoothctl prompt:
   ```
   sudo bluetoothctl
   agent on
   default-agent
   scan on
   ** note MAC address of XBox controller.  Will be XX:XX:XX:XX:XX:XX in future steps**
   scan off
   pair XX:XX:XX:XX:XX:XX
   trust XX:XX:XX:XX:XX:XX
   exit
7. Controller should change to solid light after `pair` step.  Controller will reconnect on reboot.  Use `jstest /dev/input/js0` to test joystick inputs (from `sudo apt install joystick` package)

### References:

* [Main steps](https://pimylifeup.com/xbox-controllers-raspberry-pi/)
* [bthelper instructions](https://retropie.org.uk/forum/topic/28560/xbox-series-x-controller-wont-pair-with-rp4/36)
* [Mostly complete steps](https://retropie.org.uk/forum/topic/28560/xbox-series-x-controller-wont-pair-with-rp4/16)

## Attribution

Content from the following external sources is used within this project:

* [wpilibsuite/allwpilib](https://github.com/wpilibsuite/allwpilib/tree/v2021.3.1)
* [CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example/tree/9ed6d7f370c6effea245790cd5c5d393a3f26a6a)
* [FRC1756-Argos/ArgosLib-Cpp](https://github.com/FRC1756-Argos)
* [tttapa/RPi-CPP-Toolchain](https://github.com/tttapa/RPi-Cpp-Toolchain/tree/103eb26a24a7e03b9672638ece72881311a6c9df)
* [SDL2](https://www.libsdl.org/download-2.0.php)
