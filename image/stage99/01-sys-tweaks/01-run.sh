#!/bin/bash -e

install -m 644 files/bthelper@hci0.override	"${ROOTFS_DIR}/etc/systemd/system/bthelper@hci0.service.d/override.conf"
install -m 644 ../../../scripts/swerve-platform.service "${ROOTFS_DIR}/etc/systemd/system/"

cat files/config.txt >> "${ROOTFS_DIR}/boot/config.txt"

systemctl enable swerve-platform.service

mkdir -p "${ROOTFS_DIR}/home/${FIRST_USER_NAME}/swerve-platform"

cp -r ../../../scripts ../../../build/bin ../../../build/lib "${ROOTFS_DIR}/home/${FIRST_USER_NAME}/swerve-platform/"
chown 1000:1000 -R "${ROOTFS_DIR}/home/${FIRST_USER_NAME}/swerve-platform"
