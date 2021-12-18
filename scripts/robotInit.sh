#!/bin/bash

sudo /home/pi/scripts/canInit.sh

logFile=/home/pi/log/"$(date +"%FT%H%M%S").log"
sudo /home/pi/bin/PlatformApp &> $logFile &
