#!/usr/bin/env python

import rospy
import tf
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tft
import numpy as np

def odom_callback(msg):
    try:
        # Create the transformation matrix from the static transform parameters
        translation = [args.x, args.y, args.z]
        rotation = tft.quaternion_from_euler(args.roll, args.pitch, args.yaw)
        transform_matrix = tft.concatenate_matrices(
            tft.translation_matrix(translation),
            tft.quaternion_matrix(rotation)
        )

        # Extract the position and orientation from the Odometry message
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 1.0])
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # Transform the position
        transformed_position = np.dot(transform_matrix, position)

        # Transform the orientation
        transformed_orientation = tft.quaternion_multiply(rotation, orientation)

        # Create a new Odometry message with the transformed pose
        transformed_odom = Odometry()
        transformed_odom.header = msg.header
        transformed_odom.child_frame_id = msg.child_frame_id
        transformed_odom.pose.pose.position.x = transformed_position[0]
        transformed_odom.pose.pose.position.y = transformed_position[1]
        transformed_odom.pose.pose.position.z = transformed_position[2]
        transformed_odom.pose.pose.orientation.x = transformed_orientation[0]
        transformed_odom.pose.pose.orientation.y = transformed_orientation[1]
        transformed_odom.pose.pose.orientation.z = transformed_orientation[2]
        transformed_odom.pose.pose.orientation.w = transformed_orientation[3]
        transformed_odom.twist = msg.twist

        # Publish the transformed odometry
        odom_pub.publish(transformed_odom)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available yet")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Transform /fastlio_odom to robot_odom frame.')
    parser.add_argument('x', type=float, help='Translation in x')
    parser.add_argument('y', type=float, help='Translation in y')
    parser.add_argument('z', type=float, help='Translation in z')
    parser.add_argument('yaw', type=float, help='Rotation in yaw')
    parser.add_argument('pitch', type=float, help='Rotation in pitch')
    parser.add_argument('roll', type=float, help='Rotation in roll')
    args = parser.parse_args()

    # -0.3 -0.1 0.0 3.14 0.0 0.0

    rospy.init_node('odom_transformer')

    # Subscribe to the /fastlio_odom topic
    rospy.Subscriber('/fastlio_odom', Odometry, odom_callback)

    # Create a publisher for the transformed odometry
    odom_pub = rospy.Publisher('/fastlio_odom_transformed', Odometry, queue_size=10)

    rospy.spin()