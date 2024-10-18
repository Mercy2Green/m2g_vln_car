import rospy
import tf
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import numpy as np

from geometry_msgs.msg import Pose, Quaternion, Point, Twist

class Odom_interface:
    def __init__(self, name, sub_topic, pub_topic, x=0.0, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0):
        self.name = name
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.translation = [x, y, z]
        self.rotation = tft.quaternion_from_euler(roll, pitch, yaw)
        self.subscriber = rospy.Subscriber(sub_topic, Odometry, self.odom_callback)
        self.publisher = rospy.Publisher(pub_topic, Odometry, queue_size=10)

    def get_sub_topic(self):
        return self.sub_topic
    
    def get_pub_topic(self):
        return self.pub_topic

    def create_tf(self):
        # Create a TransformStamped message for the static transform
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = 'odom'
        static_transform.child_frame_id = self.name
        static_transform.transform.translation.x = self.translation[0]
        static_transform.transform.translation.y = self.translation[1]
        static_transform.transform.translation.z = self.translation[2]
        static_transform.transform.rotation.x = self.rotation[0]
        static_transform.transform.rotation.y = self.rotation[1]
        static_transform.transform.rotation.z = self.rotation[2]
        static_transform.transform.rotation.w = self.rotation[3]

        return static_transform
    
    def transform_odom(self, msg):
        # Create the transformation matrix from the static transform parameters
        transform_matrix = tft.concatenate_matrices(
            tft.translation_matrix(self.translation),
            tft.quaternion_matrix(self.rotation)
        )
        try:
            # Get the transformation parameters
            translation = self.translation
            rotation = self.rotation

            # Get the current position and orientation from the odometry message
            current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            current_orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

            # Transform the position
            transformed_position = tft.translation_matrix(translation)[:3, 3] + current_position

            # Transform the orientation
            transformed_orientation = tft.quaternion_multiply(rotation, current_orientation)

            # Create a new odometry message with the transformed data
            transformed_odom = Odometry()
            transformed_odom.header = msg.header
            transformed_odom.child_frame_id = msg.child_frame_id
            transformed_odom.pose.pose.position = Point(*transformed_position)
            transformed_odom.pose.pose.orientation = Quaternion(*transformed_orientation)
            transformed_odom.twist = msg.twist

            return transformed_odom

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Transform not available yet")

        

    def transform_odom_tf(self, msg):
        # Create the transformation matrix from the static transform parameters
        transform_matrix = tft.concatenate_matrices(
            tft.translation_matrix(self.translation),
            tft.quaternion_matrix(self.rotation)
        )

        # Extract the position and orientation from the Odometry message
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 1.0])
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        # Transform the position
        transformed_position = np.dot(transform_matrix, position)

        # Transform the orientation
        transformed_orientation = tft.quaternion_multiply(self.rotation, orientation)

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

        return transformed_odom

    def odom_callback(self, msg):
        transformed_odom = self.transform_odom(msg)
        self.publisher.publish(transformed_odom)

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser(description='Transform odometry data to a new frame.')
#     parser.add_argument('--name', type=str, required=True, help='Name of the sensor')
#     parser.add_argument('--topic', type=str, required=True, help='Topic to subscribe to for odometry data')
#     parser.add_argument('--pub_topic', type=str, required=True, help='Topic to publish transformed odometry data')
#     parser.add_argument('--x', type=float, default=0.0, help='Translation in x')
#     parser.add_argument('--y', type=float, default=0.0, help='Translation in y')
#     parser.add_argument('--z', type=float, default=0.0, help='Translation in z')
#     parser.add_argument('--yaw', type=float, default=0.0, help='Rotation in yaw')
#     parser.add_argument('--pitch', type=float, default=0.0, help='Rotation in pitch')
#     parser.add_argument('--roll', type=float, default=0.0, help='Rotation in roll')
#     args = parser.parse_args()

#     rospy.init_node('odom_interface_node')
#     odom_interface = Odom_interface(
#         name=args.name,
#         topic=args.topic,
#         pub_topic=args.pub_topic,
#         x=args.x, y=args.y, z=args.z, yaw=args.yaw, pitch=args.pitch, roll=args.roll
#     )
#     rospy.spin()

#     # Example command to run the script:
#     # rosrun your_package_name odom_interface.py --name sensor1 --topic /sensor1/odom --pub_topic /transformed_odom/sensor1 --x 0.0 --y 0.0 --z 0.0 --yaw 0.0 --pitch 0.0 --roll 0.0