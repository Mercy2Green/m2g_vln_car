#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

class GimbalCommander:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gimbal_commander', anonymous=True)

        # Publishers to send target angles
        self.horizontal_target_angle_pub = rospy.Publisher('gimbla_control_horizontal_angle', Float32, queue_size=1)
        self.vertical_target_angle_pub = rospy.Publisher('gimbla_control_vertical_angle', Float32, queue_size=1)

        # Subscribers to receive current angles
        self.horizontal_angle_sub = rospy.Subscriber('gimbla_horizontal_angle', Float32, self.horizontal_angle_callback)
        self.vertical_angle_sub = rospy.Subscriber('gimbla_vertical_angle', Float32, self.vertical_angle_callback)

        self.target_horizontal_angle = None
        self.target_vertical_angle = None
        self.current_horizontal_angle = None
        self.current_vertical_angle = None

    def horizontal_angle_callback(self, msg):
        self.current_horizontal_angle = msg.data
        rospy.loginfo(f"Current Horizontal Angle: {self.current_horizontal_angle}")

    def vertical_angle_callback(self, msg):
        self.current_vertical_angle = msg.data
        #rospy.loginfo(f"Current Vertical Angle: {self.current_vertical_angle}")

    def send_target_angles(self, horizontal_angle, vertical_angle):
        self.target_horizontal_angle = horizontal_angle
        self.target_vertical_angle = vertical_angle
        self.horizontal_target_angle_pub.publish(self.target_horizontal_angle)
        self.vertical_target_angle_pub.publish(self.target_vertical_angle)
        rospy.loginfo(f"Sent Target Horizontal Angle: {self.target_horizontal_angle}")
        #rospy.loginfo(f"Sent Target Vertical Angle: {self.target_vertical_angle}")

    def check_target_reached(self):
        if self.target_horizontal_angle is None or self.target_vertical_angle is None:
            return False
        return (self.current_horizontal_angle == self.target_horizontal_angle and
                self.current_vertical_angle == self.target_vertical_angle)

if __name__ == '__main__':
    commander = GimbalCommander()
    rate = rospy.Rate(2)  # 1 Hz

    while not rospy.is_shutdown():
        # Example: Send target angles of 45 degrees for horizontal and 30 degrees for vertical
        commander.send_target_angles(130, 0)

        # Check if the target angles have been reached
        if commander.check_target_reached():
            rospy.loginfo("Target angles have been reached.")
        else:
            rospy.loginfo("Target angles have not been reached yet.")

        rate.sleep()