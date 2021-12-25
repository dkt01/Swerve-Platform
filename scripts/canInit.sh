#!/bin/bash

if [ "$EUID" -ne 0 ]
then
  echo "[ERROR] Run this script using sudo"
  exit 1
fi

interface=can0
if [ $# -gt 0 ]; then
    interface=$1
fi

ip link set $interface type can bitrate 1000000
ifconfig $interface up
ifconfig can0 txqueuelen 1000
