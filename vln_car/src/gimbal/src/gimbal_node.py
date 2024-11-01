#!/usr/bin/env python

from time import sleep
import debugpy
debugpy.listen(2458)
# debugpy.wait_for_client()

import rospy
import threading
from std_msgs.msg import Float32, Float32MultiArray, Int8
import serial

class GimblaNode:

    # For this node, we have two main funcitonality:
    # The first is read a command angle and send the turn command to the gimbal.
    # The second is to check the current angle of the gimbal whether it is turn to the correct angle.
    # The third is to publish the angle to the topic. 

    def __init__(
            self,
            portx="/dev/ttyUSB0",
            bps=2400,
            timex=5,
            address=1
            ):
        rospy.init_node('GimblaNode', anonymous=True)
        
        # Serial port setup
        self.portx = portx
        self.bps = bps
        self.timex = timex
        self.ser = serial.Serial(self.portx, self.bps, timeout=self.timex)
        self.address = address

        buffer_length = 3
        frame_length = 7

        self.buffer = bytearray(buffer_length*frame_length)  # Buffer to accumulate received data
        self.frame = None
        self.frame_lock = threading.Lock()

        self.target_angle = None
        
        # Publisher
        self.horizontal_angle_pub = rospy.Publisher('/gimbal/h_angle', Float32, queue_size=10)
        # self.vertical_angle_pub = rospy.Publisher('gimbla_vertical_angle', Float32, queue_size=10)
        # self.target_angle_pub = rospy.Publisher('gimbla_target_angle', Float32, queue_size=1)

        self.control_horiz_angle_sub = rospy.Subscriber('/gimbal/h_control', Float32, self.get_target_and_turn, callback_args='horizontal', queue_size=5)
        
        # Start threads
        self.start_gimbla()

    def start_gimbla(self):

        rate = rospy.Rate(1)

        rospy.loginfo("Starting gimbla node")

        read_port_thread = threading.Thread(target=self.read_from_port, args=(self.ser,))
        # read_port_thread.daemon = True
        read_port_thread.start()

        publish_rate = 0.5
        angle_pub_thread = threading.Thread(target=self.get_angle_and_publish, args=(publish_rate, self.horizontal_angle_pub, self.address))
        # angle_pub_thread.daemon = True
        angle_pub_thread.start()

        while not rospy.is_shutdown():
            rate.sleep()

    def get_target_and_turn(self, msg, axis = 'horizontal'):
        if msg.data is not None:
                self.target_angle = msg.data
                target_angle = self.target_angle
                if target_angle is not None:
                    # control_data_frame = frame_control_angle(angle=target_angle, axis=axis)
                    control_data_frame = frame_control_horizontal_angle(angle=target_angle, address=self.address)
                    with self.frame_lock:
                        self.ser.reset_output_buffer()
                        self.ser.write(control_data_frame)
                    sleep(0.1)

    def check_angle_if_in_place(self, msg):
        # The first angle is horizontal angle, the second angle is vertical angle
        h_angle, v_angle = msg.data
        h_data_frame = frame_check_angle(axis='horizontal', address=self.address)
        self.ser.write(h_data_frame)
        v_data_frame = frame_check_angle(axis='vertical', address=self.address)
        self.ser.write(v_data_frame)

    def get_angle_and_check(self, target_angle = None, acc = 0.05, sub=False):
        if sub == False:
            angle_range = [target_angle-acc, target_angle+acc]
        else:
            target_angle = self.target_angle
            if target_angle is not None:
                angle_range = [target_angle-acc, target_angle+acc]
            else:
                return False
        if self.frame:
            frame = self.frame
            cur_angle = get_angle_from_frame(frame)
            if cur_angle is not None and cur_angle >= angle_range[0] and cur_angle <= angle_range[1]:
                print(cur_angle)
                return True
            else:
                return False
                
    def get_angle_and_publish(self, rate_hz, h_pub, device=1):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            data_frame = frame_check_angle(axis='horizontal', address=device)
            with self.frame_lock:
                self.ser.reset_output_buffer()
                self.ser.write(data_frame)
            if self.frame:
                # with self.frame_lock:
                frame = self.frame
                if frame[3] == 0x59:
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        h_pub.publish(angle)
                    else:
                        rospy.logwarn("Failed to get horizontal angle from frame")
                rate.sleep()
            else:
                rate.sleep()
    
    def read_from_port(self, ser):
        while not rospy.is_shutdown():
            if ser.in_waiting:
                with self.frame_lock:
                    data = ser.read(ser.in_waiting)
                    self.buffer.extend(data)  # Add received data to the buffer
                    while len(self.buffer) >= 7:  # Assuming a frame is 7 bytes long
                        if self.buffer[0] == 0xFF:  # Check for the start of the frame
                            frame = self.buffer[:7]  # Extract the frame
                            self.buffer = self.buffer[7:]  # Remove the extracted frame from the buffer
                            self.frame = frame
                        else:
                            self.buffer.pop(0)  # Remove the first byte if it's not the start of a frame

    def send_check_commands(self, rate_hz=2, horizontal_flag = True, vertical_flag = True, device=1):
        while not rospy.is_shutdown():
            rate = rospy.Rate(rate_hz) # times per second
            if horizontal_flag:
                data_frame = frame_check_angle(axis='horizontal', address=device)
                self.ser.write(data_frame)
                # print('horizontal check command: ', data_frame.hex().upper())
                rate.sleep()


    def run(self):
        rospy.spin()
        self.ser.close()

#### communication part


#### frame part

# Data frame is FF add 00 4B data1 data2 checksum
# add is the address of the deivce, the default value is 01
# data1 is the high 8 byte of the angle
# data2 is the low 8 byte of the angle
# checksum is the low 8 byte of the sum of 2 to 6 byte

def frame_control_horizontal_angle(angle, address=1):
    # the angle is the degree of the gimbal, it is a decimal number
    angle_int = int(angle*100) 
    data1 = (angle_int >> 8)
    data2 = angle_int & 0xFF
    hex_address = int(address)
    checksum = calculate_checksum(hex_address, 0x00, 0x4B, data1, data2)
    return bytes([0xFF, hex_address, 0x00, 0x4B, data1, data2, checksum])

def frame_control_vertical_angle(angle, address=1):
    # the angle is the degree of the gimbal, it is a decimal number
    angle_int = int(angle*100) 
    data1 = (angle_int >> 8) & 0xFF
    data2 = angle_int & 0xFF
    hex_address = int(address)
    checksum = calculate_checksum(hex_address, 0x00, 0x4D, data1, data2)
    return bytes([0xFF, hex_address, 0x00, 0x4D, data1, data2, checksum])

def frame_check_horizontal_angle(address=1):
    hex_address = int(address)
    return bytes([0xFF, hex_address, 0x00, 0x51, 0x00, 0x00, 0x52])

def frame_check_vertical_angle(address=1):
    hex_address = int(address)
    return bytes([0xFF, hex_address, 0x00, 0x53, 0x00, 0x00, 0x54])

def frame_control_angle(angle, axis='horizontal', address=1):
    hex_address = int(address)
    if axis == 'horizontal':
        return frame_control_horizontal_angle(angle, hex_address)
    elif axis == 'vertical':
        return frame_control_vertical_angle(angle, hex_address)
    else:
        rospy.logwarn("Invalid angle. Please enter 'horizontal' or 'vertical'.")

def frame_check_angle(axis='horizontal', address=1):
    if axis == 'horizontal':
        return frame_check_horizontal_angle(address)
    elif axis == 'vertical':
        return frame_check_vertical_angle(address)
    else:
        rospy.logwarn("Invalid angle. Please enter 'horizontal' or 'vertical'.")

def calculate_checksum(*args):
    # the frame is a bytes object
    sum = 0
    for value in args:
        sum += value
    return sum & 0xFF

#### read part

def read_angle_from_port(ser, pub):
    buffer = bytearray()  # Buffer to accumulate received data
    while not rospy.is_shutdown():
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)  # Add received data to the buffer
            while len(buffer) >= 7:  # Assuming a frame is 7 bytes long
                if buffer[0] == 0xFF:  # Check for the start of the frame
                    frame = buffer[:7]  # Extract the frame
                    buffer = buffer[7:]  # Remove the extracted frame from the buffer
                    # Convert the frame to angle
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        # print(f"Received angle: {angle}")
                        pub.publish(angle)
                else:
                    buffer.pop(0)  # Remove the first byte if it's not the start of a frame
#### process part

def get_angle_from_frame(bytes_frame):

    axis_flag = bytes_frame[3]
    if axis_flag != 0x59 and axis_flag != 0x5B:
        rospy.logwarn("Invalid frame. Please check the frame.")
        return None
    
    # the frame is a bytes object
    data1 = bytes_frame[4]
    data2 = bytes_frame[5]
    angle_int = (data1 << 8) + data2

    # also need to check the checksum
    checksum = calculate_checksum(bytes_frame[1], bytes_frame[2], bytes_frame[3], bytes_frame[4], bytes_frame[5])
    if checksum != bytes_frame[6]:
        rospy.logwarn("Checksum mismatch")
        return None

    return angle_int / 100

if __name__ == '__main__':
    try:
        node = GimblaNode(portx="/dev/ttyUSB0", bps=2400, timex=5, address=1)
        node.run()
    except rospy.ROSInterruptException:
        pass
