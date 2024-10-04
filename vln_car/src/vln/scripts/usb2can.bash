#!/bin/bash

USB2CAN_DEVICE=/dev/ttyUSB2

# enable kernel module: gs_usb
sudo modprobe gs_usb

# # bring up can interface
# sudo ip link set can0 up type can bitrate 500000

# sudo ip link set can0 down

# Set the baud rate of the USB2CAN device
# sudo stty -F $USB2CAN_DEVICE 9600

# Attach the USB2CAN device
sudo slcan_attach -s6 -o $USB2CAN_DEVICE

# Bring up the renamed CAN interface
sudo ip link set can0 up

# ip -d -s link show can0
