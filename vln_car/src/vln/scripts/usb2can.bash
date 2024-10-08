#!/bin/bash

USB2CAN_DEVICE=/dev/ttyACM0

# Enable kernel module: gs_usb
sudo modprobe gs_usb

sudo ip link set can0 down

# Attach the USB2CAN device
sudo slcan_attach -s6 -o $USB2CAN_DEVICE
sudo slcand -o -c -s6 $USB2CAN_DEVICE can0

if ! ip link show can0 > /dev/null 2>&1; then
    echo "Error: can0 interface not found."
    # Rename slcan0 to can0
    sudo ip link set slcan0 name can0
    sudo ip link set can0 up
    echo "can0 interface found."
else 
    echo "can0 interface found."
    # Bring up the renamed CAN interface
    sudo ip link set can0 up
fi