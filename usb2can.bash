#!/bin/bash

USB2CAN_DEVICE=/dev/ttyUSB2

# Enable kernel modules
sudo modprobe gs_usb
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe slcan
sudo modprobe usbserial


# # Set the baud rate of the USB2CAN device
sudo stty -F $USB2CAN_DEVICE 9600

sudo ip link set can0 down
# Attach the USB2CAN device with a bitrate of 500,000
# sudo slcan_attach -s6 -o $USB2CAN_DEVICE
sudo slcand -o -c -f -s 6 -S 9600 $USB2CAN_DEVICE can0

# Rename slcan0 to can0 if it doesn't already exist
if ! ip link show can0 > /dev/null 2>&1; then
    sudo ip link set slcan0 name can0
else
    echo "can0 interface already exists"
fi

# Bring up the renamed CAN interface if it's not already up
if ! ip link show can0 | grep -q "state UP"; then
    sudo ip link set can0 up
else
    echo "can0 interface is already up"
fi

# Display the interface details
ip -d -s link show can0

# #!/bin/bash

# USB2CAN_DEVICE=/dev/ttyUSB0

# # Enable kernel modules
# sudo modprobe gs_usb
# sudo modprobe can
# sudo modprobe can_raw
# sudo modprobe can_dev
# sudo modprobe slcan
# sudo modprobe usbserial


# # Set the baud rate of the USB2CAN device
# sudo stty -F $USB2CAN_DEVICE 9600

# # Attach the USB2CAN device with a bitrate of 500,000
# sudo slcan_attach -s6 -o $USB2CAN_DEVICE
# sudo slcand -o -s6 $USB2CAN_DEVICE can0

# # Check if slcan0 was created
# if ip link show slcan0 > /dev/null 2>&1; then
#     echo "slcan0 interface created successfully"
# else
#     echo "Failed to create slcan0 interface"
#     exit 1
# fi

# # Rename slcan0 to can0 if it doesn't already exist
# if ! ip link show can0 > /dev/null 2>&1; then
#     sudo ip link set slcan0 name can0
# else
#     echo "can0 interface already exists"
# fi

# # Bring up the renamed CAN interface if it's not already up
# if ! ip link show can0 | grep -q "state UP"; then
#     sudo ip link set can0 up
# else
#     echo "can0 interface is already up"
# fi

# # Display the interface details
# ip -d -s link show can0