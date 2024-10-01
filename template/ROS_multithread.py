#!/usr/bin/env python

import rospy
import threading
from std_msgs.msg import Float32
import serial
from communication import read_from_port_ros, frame_check_horizontal_angle

class MultiThreadedNode:
    def __init__(self):
        rospy.init_node('multi_threaded_node', anonymous=True)
        
        # Serial port setup
        self.portx = "/dev/ttyUSB1"
        self.bps = 2400
        self.timex = 5
        self.ser = serial.Serial(self.portx, self.bps, timeout=self.timex)
        
        # Publisher
        self.angle_pub = rospy.Publisher('received_angle', Float32, queue_size=10)
        
        # Start threads
        self.start_threads()

    def start_threads(self):
        # Thread for reading from the serial port
        read_thread = threading.Thread(target=read_from_port_ros, args=(self.ser, self.angle_pub))
        read_thread.daemon = True
        read_thread.start()

        # Thread for sending check commands
        check_thread = threading.Thread(target=self.send_check_commands)
        check_thread.daemon = True
        check_thread.start()

        # Thread for handling user input
        input_thread = threading.Thread(target=self.handle_user_input)
        input_thread.daemon = True
        input_thread.start()

    def send_check_commands(self):
        rate = rospy.Rate(0.2)  # 5 seconds
        while not rospy.is_shutdown():
            data_frame = frame_check_horizontal_angle()
            self.ser.write(data_frame)
            rospy.loginfo("Sent check command: %s", data_frame.hex().upper())
            rate.sleep()

    def handle_user_input(self):
        while not rospy.is_shutdown():
            user_input = input("Enter a command: ")
            rospy.loginfo(f"User input received: {user_input}")
            # Process user input here

    def run(self):
        rospy.spin()
        self.ser.close()

if __name__ == '__main__':
    try:
        node = MultiThreadedNode()
        node.run()
    except rospy.ROSInterruptException:
        pass


import rospy
import serial
import threading
from time import sleep
from std_msgs.msg import Float32, Float32MultiArray
from communication import read_from_port, frame_check_angle, frame_control_angle, get_angle_from_frame

class GimbalNode:
    def __init__(self):
        rospy.init_node('gimbal_node', anonymous=True)
        self.portx = "/dev/ttyUSB1"
        self.bps = 2400
        self.timex = 5
        self.ser = serial.Serial(self.portx, self.bps, timeout=self.timex)
        self.angle_pub = rospy.Publisher('received_angle', Float32, queue_size=10)
        self.angle_sub = rospy.Subscriber('control_angle', Float32MultiArray, self.receive_angle_and_turn)
        self.buffer_length = 10  # Specify the desired buffer length
        self.buffer = bytearray(self.buffer_length)  # Initialize the buffer with the specified length
        self.frame = None  # Initialize frame
        self.frame_lock = threading.Lock()  # Create a lock for the frame
        self.start_threads()

    def start_threads(self):
        read_thread = threading.Thread(target=read_from_port, args=(self.ser, self.angle_pub))
        read_thread.daemon = True
        read_thread.start()

        check_thread = threading.Thread(target=self.send_check_commands)
        check_thread.daemon = True
        check_thread.start()

    def send_check_commands(self, rate=2, horizontal_flag=True, vertical_flag=True, device=1):
        while not rospy.is_shutdown():
            if horizontal_flag:
                data_frame = frame_check_angle(angle='horizontal', address=device)
                self.ser.write(data_frame)
                sleep(rate)
            if vertical_flag:
                data_frame = frame_check_angle(angle='vertical', address=device)
                self.ser.write(data_frame)
                sleep(rate)

    def receive_angle_and_turn(self, msg):
        angles = msg.data
        rospy.loginfo(f"Received angles: {angles}")
        # Process the received angles here

    def handle_user_input(self):
        while not rospy.is_shutdown():
            user_input = input("Enter a command: ")
            rospy.loginfo(f"User input received: {user_input}")
            # Process user input here

    def test_control_angle(self, axis='horizontal'):
        """
        This function waits for user input angle from the keyboard and sends the control command to the gimbal.
        """
        while not rospy.is_shutdown():
            try:
                angle = float(input(f"Enter the {axis} angle to control the gimbal: "))
                data_frame = frame_control_angle(angle=angle, axis=axis)
                self.ser.write(data_frame)
                rospy.loginfo(f"Sent control command for {axis} angle: {angle}")
            except ValueError:
                rospy.logerr("Invalid input. Please enter a valid number.")

    def check_angle_if_in_place(self, msg):
        # The first angle is horizontal angle, the second angle is vertical angle
        h_angle, v_angle = msg.data
        h_data_frame = frame_check_angle(axis='horizontal', address=self.address)
        self.ser.write(h_data_frame)
        v_data_frame = frame_check_angle(axis='vertical', address=self.address)
        self.ser.write(v_data_frame)
    
    def get_angle_and_publish(self, h_pub, v_pub, horizontal_flag=True, vertical_flag=True):
        while not rospy.is_shutdown():
            with self.frame_lock:  # Acquire the lock before accessing self.frame
                if self.frame:
                    if self.frame[3] == 0x59 and horizontal_flag:
                        angle = get_angle_from_frame(self.frame)
                        if angle is not None:
                            h_pub.publish(angle)
                            # print('horizontal angle: ', angle)
                        else:
                            rospy.logwarn("Failed to get horizontal angle from frame")
                    if self.frame[3] == 0x5B and vertical_flag:
                        angle = get_angle_from_frame(self.frame)
                        if angle is not None:
                            v_pub.publish(angle)
                            # print('vertical angle: ', angle)
                        else:
                            rospy.logwarn("Failed to get vertical angle from frame")

    def run(self):
        rospy.spin()
        self.ser.close()

if __name__ == '__main__':
    try:
        node = GimbalNode()
        node.run()
    except rospy.ROSInterruptException:
        pass