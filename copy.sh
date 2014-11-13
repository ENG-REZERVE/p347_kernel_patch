#!/bin/bash
mount -t vfat /dev/sda1 /mnt/flash
cp /mnt/flash/drv/* /mnt/share
umount /dev/sda1
