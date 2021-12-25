#!/bin/bash

# Designed to run as systemd service and output logged to journalctl

if [ "$EUID" -ne 0 ]
then
  echo "[ERROR] Run this script using sudo"
  exit 1
fi

APPLICATION_DIR="/home/pi/swerve-platform"

"${APPLICATION_DIR}/scripts/canInit.sh"

LD_LIBRARY_PATH="${APPLICATION_DIR}/lib" "${APPLICATION_DIR}/bin/PlatformApp"
