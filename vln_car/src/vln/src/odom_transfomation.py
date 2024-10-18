import rospy
import tf
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, Twist
import tf.transformations as tft

def odom_callback(msg):
    try:
        # Get the transformation parameters
        translation = [args.x, args.y, args.z]
        rotation = tft.quaternion_from_euler(args.roll, args.pitch, args.yaw)

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

    rospy.init_node('odom_transformer')

    # Subscribe to the /fastlio_odom topic
    rospy.Subscriber('/fastlio_odom', Odometry, odom_callback)

    # Create a publisher for the transformed odometry
    odom_pub = rospy.Publisher('/fastlio_odom_transformed', Odometry, queue_size=10)

    rospy.spin()