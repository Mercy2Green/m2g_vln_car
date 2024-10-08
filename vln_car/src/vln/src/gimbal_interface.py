import rospy
import actionlib
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, Quaternion
from std_msgs.msg import Float32, Float32MultiArray

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Gimbal_interface(object):

    def __init__(
        self,
        gimbal_name,
        gimbal_horiz_control_topic = '/gimbal/horiz_control',
        gimbal_vert_control_topic = '/gimbal/vert_control',
        gimbal_horiz_angle_topic = '/gimbal/horiz_angle',
        gimbal_vert_angle_topic = '/gimbal/vert_angle',
        ):

        self.gimbal_name = gimbal_name
        self.horiz_angle = 0
        self.vert_angle = 0
        self.target_horiz_angle = None
        self.target_vert_angle = None

        self.horiz_angle_range = [0, 350]
        self.vert_angle_range = [-60, 20]

        self.pub_gimbal_horiz_control = rospy.Publisher(
            # name="/" + self.gimbal_name + "/gimbal/horiz_control",
            name = '/' + self.gimbal_name + gimbal_horiz_control_topic,
            data_class=Float32,
            queue_size=1,
        )

        self.pub_gimbal_vert_control = rospy.Publisher(
            # name="/" + self.gimbal_name + "/gimbal/vert_control",
            name = '/' + self.gimbal_name + gimbal_vert_control_topic,
            data_class=Float32,
            queue_size=1,
        )

        self.sub_gimbal_horiz_angle = rospy.Subscriber(
            # name = '/' + self.gimbal_name + '/gimbal/horiz_angle',
            name = '/' + self.gimbal_name + gimbal_horiz_angle_topic,
            data_class=Float32,
            callback=self.gimbal_horiz_angle_cb,
        )

        self.sub_gimbal_vert_angle = rospy.Subscriber(
            # name = '/' + self.gimbal_name + '/gimbal/vert_angle',
            name = '/' + self.gimbal_name + gimbal_vert_angle_topic,
            data_class=Float32,
            callback=self.gimbal_vert_angle_cb,
        )

    def gimbal_horiz_angle_cb(self, msg):
        self.horiz_angle = msg.data

    def gimbal_vert_angle_cb(self, msg):
        self.vert_angle = msg.data

    def control_gimbal_horiz_angle(self, angle):
        self.pub_gimbal_horiz_control.publish(angle)

    def control_gimbal_vert_angle(self, angle):
        self.pub_gimbal_vert_control.publish(angle)

    def pan_tilt_move(self, pan_angle, tilt_angle):
        rate = rospy.Rate(0.5)
        if pan_angle is not None:
            if pan_angle >= self.horiz_angle_range[0] and pan_angle <= self.horiz_angle_range[1]:
                self.control_gimbal_horiz_angle(pan_angle)
                rate.sleep()
            else:
                rospy.logwarn("Pan angle out of range: " + str(pan_angle))
        if tilt_angle is not None:
            if tilt_angle >= self.vert_angle_range[0] and tilt_angle <= self.vert_angle_range[1]:
                self.control_gimbal_vert_angle(tilt_angle)
                rate.sleep()
            else:
                rospy.logwarn("Tilt angle out of range: " + str(tilt_angle))

# class Gimbla_Server(object):

#     def __init__(
#             self,
#             control_topic = 'gimbla_target_angle',
#             horizontal_angle_topic = 'gimbla_horizontal_angle') -> None:
        
#         self.control_topic = control_topic
#         self.horizontal_angle_topic = horizontal_angle_topic

#         self.horizontal_angle = 0
#         self.target_angle = None

#         self.control_pub = rospy.Publisher('/gimbla_target_angle', Float32, queue_size=2)
#         self.horizontal_angle_sub = rospy.Subscriber('/gimbla_target_angle', Float32, self.horizontal_angle_callback)

#     def horizontal_angle_callback(self, msg):
#         self.horizontal_angle = msg.data

#     def control_angle(self, angle):
#         self.control_pub.publish(angle)

# class Gimbla_client(object):

#     def __init__(
#             self,
#             control_topic = 'gimbla_target_angle',
#             horizontal_angle_topic = 'gimbla_horizontal_angle') -> None:
        
#         self.control_topic = control_topic
#         self.horizontal_angle_topic = horizontal_angle_topic

#         self.horizontal_angle = 0
#         self.target_angle = None

#         self.control_pub = rospy.Publisher('/gimbla_target_angle', Float32, queue_size=2)
#         self.horizontal_angle_sub = rospy.Subscriber('/gimbla_target_angle', Float32, self.horizontal_angle_callback)

#     def horizontal_angle_callback(self, msg):
#         self.horizontal_angle = msg.data

#     def control_angle(self, angle):
#         self.control_pub.publish(angle)