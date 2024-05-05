#!/bin/bash

xhost +

usb_info=$(lsusb | grep 'Intel Corp.')

if [ -n "$usb_info" ]; then
    bus=$(echo $usb_info | awk '{print $2}')
    device=$(echo $usb_info | awk '{print $4}' | tr -d ':')
    
    sudo_pw="jetson"
    echo $sudo_pw | sudo -S ../system/usbreset /dev/bus/usb/$bus/$device
else
    echo "No Intel Corp. device found."
fi


