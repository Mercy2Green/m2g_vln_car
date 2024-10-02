#!/usr/bin/env python

from time import sleep
import debugpy

# debugpy.listen(5678)
# debugpy.wait_for_client()

import rospy
import threading
from std_msgs.msg import Float32, Float32MultiArray
import serial

class GimblaNode:

    # For this node, we have two main funcitonality:
    # The first is read a command angle and send the turn command to the gimbal.
    # The second is to check the current angle of the gimbal whether it is turn to the correct angle.
    # The third is to publish the angle to the topic. 

    def __init__(
            self,
            portx="/dev/ttyUSB1",
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
        self.horizontal_angle_pub = rospy.Publisher('gimbla_horizontal_angle', Float32, queue_size=10)
        self.vertical_angle_pub = rospy.Publisher('gimbla_vertical_angle', Float32, queue_size=10)
        self.target_angle_pub = rospy.Publisher('gimbla_target_angle', Float32, queue_size=1)

        self.turn_sub = rospy.Subscriber('control_angle', Float32MultiArray, self.receive_angle_and_turn)
        self.target_angle_sub = rospy.Subscriber('gimbla_target_angle', Float32, self.current_target_angle)
        # the angle is a list of two elements, the first is the horizontal angle, the second is the vertical angle
        
        # Start threads
        self.start_gimbla()

    def start_gimbla(self):

        rospy.loginfo("Starting gimbla node")

        read_port_thread = threading.Thread(target=self.read_from_port, args=(self.ser,))
        read_port_thread.daemon = True
        read_port_thread.start()

        angle_pub_thread = threading.Thread(target=self.get_angle_and_publish, args=(self.horizontal_angle_pub, self.vertical_angle_pub))
        angle_pub_thread.daemon = True
        angle_pub_thread.start()

        while not rospy.is_shutdown():

            target_angle = self.test_control_angle(axis='horizontal')
            self.target_angle_pub.publish(target_angle)
            sleep(0.5) # This is essential cause the device need time to fresh the self.target_angle
            if target_angle is not None:
                self.target_angle_pub.publish(target_angle)
                while not self.get_angle_and_check(target_angle, 0.05, True):
                    # self.target_angle_pub.publish(target_angle)
                    control_data_frame = frame_control_angle(angle=target_angle, axis='horizontal')
                    self.ser.write(control_data_frame)
                    check_angle_frame = frame_check_angle(axis='horizontal', address=self.address)
                    self.ser.write(check_angle_frame)
                    sleep(0.5)
            else:
                continue
                

    def current_target_angle(self, msg):
        if msg.data is not None:
            self.target_angle = msg.data

    def receive_angle_and_turn(self, msg): 
        # The first angle is horizontal angle, the second angle is vertical angle
        h_angle, v_angle = msg.data
        h_data_frame = frame_control_angle(h_angle, axis='horizontal', address=self.address)
        self.ser.write(h_data_frame)
        v_data_frame = frame_control_vertical_angle(v_angle, axis='vertical', address=self.address)
        self.ser.write(v_data_frame)
    
    def test_control_angle(self, axis='horizontal'):
        """
        This function waits for user input angle from the keyboard and sends the control command to the gimbal.
        """
        try:
            angle = float(input(f"Enter the {axis} angle to control the gimbal: "))
            data_frame = frame_control_angle(angle=angle, axis=axis)
            self.ser.write(data_frame)
            rospy.loginfo(f"Sent control command for {axis} angle: {angle}")
            return angle
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid number.")
            return None

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
                
    def get_angle_and_publish(self, h_pub, v_pub, horizontal_flag = True, vertical_flag = True):
        while not rospy.is_shutdown():
            if self.frame:
                frame = self.frame
                if frame[3] == 0x59 and horizontal_flag:
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        h_pub.publish(angle)
                        # print('horizontal angle: ', angle)
                    else:
                        rospy.logwarn("Failed to get horizontal angle from frame")
                if frame[3] == 0x5B and vertical_flag:
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        v_pub.publish(angle)
                        # print('vertical angle: ', angle)
                    else:
                        rospy.logwarn("Failed to get vertical angle from frame")
    
    def read_from_port(self, ser):
        while not rospy.is_shutdown():
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                # print(data)
                self.buffer.extend(data)  # Add received data to the buffer
                while len(self.buffer) >= 7:  # Assuming a frame is 7 bytes long
                    if self.buffer[0] == 0xFF:  # Check for the start of the frame
                        frame = self.buffer[:7]  # Extract the frame
                        self.buffer = self.buffer[7:]  # Remove the extracted frame from the buffer
                        self.frame = frame
                    else:
                        self.buffer.pop(0)  # Remove the first byte if it's not the start of a frame

    def send_check_commands(self, rate=2, horizontal_flag = True, vertical_flag = True, device=1):
        while not rospy.is_shutdown():
            if horizontal_flag:
                data_frame = frame_check_angle(angle='horizontal', address=device)
                self.ser.write(data_frame)
                # print('horizontal check command: ', data_frame.hex().upper())
                sleep(rate)
            if vertical_flag:
                data_frame = frame_check_angle(angle='vertical', address=device)
                self.ser.write(data_frame)
                sleep(rate)
                # print('vertical check command: ', data_frame.hex().upper())
            # rospy.loginfo("Sent check command: %s", data_frame.hex().upper())

    def handle_user_input(self):
        while not rospy.is_shutdown():
            user_input = input("Enter a command: ")
            rospy.loginfo(f"User input received: {user_input}")
            # Process user input here

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
    data1 = (angle_int >> 8) & 0xFF
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
        Warning("Checksum mismatch")
        return None

    return angle_int / 100

if __name__ == '__main__':
    try:
        node = GimblaNode(portx="/dev/ttyUSB0", bps=2400, timex=5, address=1)
        node.run()
    except rospy.ROSInterruptException:
        pass
