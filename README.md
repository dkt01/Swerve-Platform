# Swerve-Platform

[![build-ci](https://github.com/dkt01/Swerve-Platform/actions/workflows/build-ci.yml/badge.svg)](https://github.com/dkt01/Swerve-Platform/actions/workflows/build-ci.yml)

Robotic swerve drive platform developed for Northwoods Church Christmas program

## Controls

Control is all using an XBox Series wireless controller.

### Driving Inputs

| Input | Control Description |
| ----- | ------------------- |
| <kbd>RB</kbd> | Hold to enable driving |
| Left Joystick Forward/Reverse | Drive forward/reverse |
| Left Joystick Left/Right | Drive left/right |
| Right Joystick Left/Right | Rotate counter-clockwise/clockwise |
| <kbd>LT</kbd> + <kbd>RT</kbd> | Hold for 2 seconds to enable module homing |
| <kbd>A</kbd> | When in homing mode, hold for 1 second to save new home position |

### Vibration Feedback

The driver controller will vibrate to indicate various events.

| Vibration Pattern | Description |
| ----------------- | ----------- |
| Faint rumble with no buttons/joysticks moved | XBox Controller connected to Raspberry Pi after controller power on or platform reboot |
| 500ms pulse | Indicates drive mode has been activated and joysticks now command platform motion.  Will occur when <kbd>RB</kbd> is pressed and joysticks are centered. |
| Rapid pulses | Indicates drive mode is locked out because joysticks are not centered.  Will occur when <kbd>RB</kbd> is pressed while joysticks are not centered. |
| Continuous waves | Indicates homing mode is primed.  Will occur when <kbd>LT</kbd> and <kbd>RT</kbd> are both held for at least two seconds. |
| Continuous medium intensity | Indicates new home position was saved.  Will occur after homing mode is primed and <kbd>A</kbd> is held for an additional 1 second. |

### Homing

Whenever a module is swapped; a module, motor, or sensor is unmounted and remounted; or a freshly imaged Raspberry Pi controller is installed, you will need to "home" the swerve modules so they all move together properly.  Symptoms of an improperly homed platform range from the platform not driving straight to the platform moving erratically or stalling.  You should see all modules pointed in the same direction when moving using only the left joystick after homing is performed successfully.

To save a new home position, do the following:

1. Turn platform power off
2. Align all modules so the bevel gear on the wheel is facing toward the right side of the platform.  Using a square is best to ensure each module is aligned to the platform
3. Turn platform power on
4. Connect XBox Controller
5. Prime homing calibration by holding <kbd>LT</kbd> & <kbd>RT</kbd> for at least 2 seconds
6. You should feel a continuous wave vibration pattern when homing is primed
7. While continuing to hold <kbd>LT</kbd> & <kbd>RT</kbd>, press and hold <kbd>A</kbd> for at least 1 second
8. You should feel continuous steady vibration when homing has saved
9. Release all buttons and try driving to validate homing

Homing values save to `/home/pi/.config/Swerve-Platform/moduleHomes` on the Raspberry Pi, so everything is saved across power cycles.

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

## Codespaces

A GitHub codespace container is available for this project.

## Pre-Commit

This project uses [pre-commit](https://pre-commit.com/) to check code formatting before accepting commits.

First install the prerequisites:

* python3 (with pip) - [instructions](https://realpython.com/installing-python/)
  * Python 3.9.x from the [Python website](https://www.python.org/downloads/) works well.  Make sure to check the add to path option in the installer.
* pip packages:
  * You may need to add the pip install path to your shell's path if you're using Git Bash.  In git bash:
    1. Open (or create) new file `~/.bashrc` by running `vim ~/.bashrc`
    2. Add this to the end: `PATH=$PATH:$LOCALAPPDATA/Programs/Python/Python39/Scripts/` (change `Python39` to match your python version)
       * **Note**: The actual path you need to add (`$LOCALAPPDATA/Programs/Python/Python39/Scripts/` in the above example) depends on your Python installation.  If y ou do the `pip install` steps first, pip will print the path you need to add.
       * To type in Vim, type <kbd>i</kbd> and you should see `INSERT` at the bottom of the window to indicate you're editing in insert mode
    3. Exit by pressing <kbd>Esc</kbd> then type `:wq` and press <kbd>Enter</kbd>
    4. Run `source ~/.bashrc` to update your session
  * clang-format - `pip install clang-format`
  * pre-commit - `pip install pre-commit`

Then initialize:

```
pre-commit install
pre-commit run
```

The first run may take a moment, but subsequent automatic runs are very fast.

You'll now have the linter run before each commit!  For compatibility with Windows, we recommend the pip version of clang-format, but wpi-format will find any installed `clang-format` binary in the system path.

## SSH
`ssh pi@NWCC-platform-alpha.local`

## How To Run At Startup

1. Copy `bin`, `lib`, and `scripts` to `/home/pi/swerve-platform/`
2. Copy `scripts/swerve-platform.service` to `/etc/systemd/system/`: `sudo cp /home/pi/swerve-platform/scripts/swerve-platform.service /etc/systemd/system`
3. Adjust permissions of unit file: `sudo chmod 644 /etc/systemd/system/swerve-platform.service`
4. Enable service: `sudo systemctl enable swerve-platform.service`
5. Reboot

## Required Software Packages

### Raspberry Pi Image

The `image/` directory contains directives to set up an image with most settings preconfigured.  Unfortunately, this is not building correctly in GitHub actions, so the manual steps are documented here.

Start with [Raspberry Pi OS Lite](https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-legacy).  Legacy (Buster) definitely works.  Latest (Bullseye) should also work, but may have some compatibility issues.  Use 32-bit OS version if prompted.

Use [Balena Etcher](https://www.balena.io/etcher/) to write the downloaded Raspberry Pi OS image to a microSD card.

On first boot,

1. Expand the filesystem so the full SD card space is usable.  This can be done using [`raspi-config`](https://piwithvic.com/raspberry-pi-expand-filesystem-micro-sd-card)
2. [Change the hostname](https://www.tomshardware.com/how-to/raspberry-pi-change-hostname#change-raspberry-pi-hostname-at-command-prompt-xa0) to `NWCC-platform-alpha` (or your preferred hostname that will later be used for SSH access)
3. [Enable SSH access](https://phoenixnap.com/kb/enable-ssh-raspberry-pi#ftoc-heading-4)

Then reboot before continuing setup.

### Linux Platform Software

1. `sudo apt-get update`
2. `sudo apt-get upgrade`

### PiCAN

Add the following lines to `/boot/config.txt`
1. `dtparam=spi=on`
2. `dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25`
3. `dtoverlay=spi-bcm2835-overlay`

### CAN-Utils

1. `sudo apt-get install can-utils`

### xpadneo
1. `sudo apt install dkms linux-headers`
2. Reboot
3. `git clone https://github.com/atar-axis/xpadneo.git`
4. `cd xpadneo`
5. `sudo ./install.sh`

## Bare Minimum To Pair XBox Series Controller

1. Update controller firmware in Windows using [XBox Accessories App](https://www.microsoft.com/en-us/p/xbox-accessories/9nblggh30xj3)
2. `echo 'options bluetooth disable_ertm=Y' | sudo tee -a /etc/modprobe.d/bluetooth.conf`
3. `systemctl edit bthelper@hci0` and add the following content:
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
