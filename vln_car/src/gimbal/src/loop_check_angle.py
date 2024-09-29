#!/usr/bin/env python

#!/usr/bin/env python

import rospy
import serial
import threading
from std_msgs.msg import Float32
from communication import read_from_port, frame_check_horizontal_angle, read_from_port_ros

def send_check_command(event, ser):
    data_frame = frame_check_horizontal_angle()
    ser.write(data_frame)
    # rospy.loginfo("Sent check command: %s", data_frame.hex().upper())

def check_command_publisher():
    rospy.init_node('check_command_publisher', anonymous=True)
    portx = "/dev/ttyUSB0"
    bps = 2400
    timex = 5
    ser = serial.Serial(portx, bps, timeout=timex)

    pub = rospy.Publisher('received_angle', Float32, queue_size=10)

    # Create and start the reading thread
    read_thread = threading.Thread(target=read_from_port_ros, args=(ser, pub))
    read_thread.daemon = True
    read_thread.start()

    # Schedule the send_check_command function to be called every 5 seconds
    rospy.Timer(rospy.Duration(0.5), lambda event: send_check_command(event, ser))

    rospy.spin()
    ser.close()

if __name__ == '__main__':
    try:
        check_command_publisher()
    except rospy.ROSInterruptException:
        pass