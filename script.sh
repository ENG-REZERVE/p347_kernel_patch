#!/bin/bash

#Check number of input parameters
if [ $# -ne 1 ]
then
    echo 'You need to specify one parameter'
    exit
fi

case $1 in
 20) var_module="./p_20m.ko" ;;
 80-128) var_module="./p80-128.ko" ;;
 80-256) var_module="./p80-256.ko" ;;
 166) var_module="./p_166m.ko" ;;
 *) echo 'Invalid parameter' 
    exit
esac

cd /mnt/share
mknod /dev/p347_fpga_device c 241 0

insmod $var_module

