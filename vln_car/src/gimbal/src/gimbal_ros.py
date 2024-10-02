#!/usr/bin/env python


import sys
import os

from gimbal import Gimbal
import threading
import rospy
from std_msgs.msg import Float32

class GimbalController:
    def __init__(self):
        self.gimbal = Gimbal(portx='/dev/ttyUSB1', address=0x01)  # Assuming address is 0x01
        self.angle_lock = threading.Lock()
        self.current_horizontal_angle = 0.0
        self.current_vertical_angle = 0.0
        self.target_horizontal_angle = 0.0
        self.target_vertical_angle = 0.0

        rospy.init_node('gimbal_controller', anonymous=True)
        rospy.Subscriber('/gimbal/target_horizontal_angle', Float32, self.target_horizontal_angle_callback)
        rospy.Subscriber('/gimbal/target_vertical_angle', Float32, self.target_vertical_angle_callback)
        self.horizontal_angle_pub = rospy.Publisher('/gimbal/current_horizontal_angle', Float32, queue_size=10)
        self.vertical_angle_pub = rospy.Publisher('/gimbal/current_vertical_angle', Float32, queue_size=10)

        self.test_control_pub = rospy.Publisher('/gimbal/target_horizontal_angle', Float32, queue_size=2)

        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

        while not rospy.is_shutdown():

            target_angle = self.test_control_angle()
            for i in range(10):
                self.test_control_pub.publish(target_angle)
                rospy.sleep(0.1)

    def test_control_angle(self, axis='horizontal'):
        """
        This function waits for user input angle from the keyboard and sends the control command to the gimbal.
        """
        try:
            angle = float(input(f"Enter the {axis} angle to control the gimbal: "))
            # data_frame = frame_control_angle(angle=angle, axis=axis)
            # self.ser.write(data_frame)
            rospy.loginfo(f"Sent control command for {axis} angle: {angle}")
            return angle
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid number.")
            return None

    def target_horizontal_angle_callback(self, msg):
        with self.angle_lock:
            self.target_horizontal_angle = msg.data

    def target_vertical_angle_callback(self, msg):
        with self.angle_lock:
            self.target_vertical_angle = msg.data

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            with self.angle_lock:
                # Control horizontal angle
                self.gimbal.control_horizontal_angle(self.target_horizontal_angle)
                # # Control vertical angle
                # if self.current_vertical_angle < self.target_vertical_angle:
                #     self.gimbal.control_up()
                # elif self.current_vertical_angle > self.target_vertical_angle:
                #     self.gimbal.control_down()
                # else:
                #     self.gimbal.control_stop()

                # Read the current angles from the gimbal
                self.current_horizontal_angle = self.gimbal.check_horizontal_angle()
                # self.current_vertical_angle = self.gimbal.check_vertical_angle()

                # Publish the current angles
                self.horizontal_angle_pub.publish(self.current_horizontal_angle)
                # self.vertical_angle_pub.publish(self.current_vertical_angle)

            rate.sleep()

if __name__ == '__main__':
    try:
        GimbalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass