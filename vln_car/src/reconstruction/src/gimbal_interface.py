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
        gimbal_name = 'gimbal',
        gimbal_h_control_topic = '/h_control',
        gimbal_v_control_topic = '/v_control',
        gimbal_h_angle_topic = '/h_angle',
        gimbal_v_angle_topic = '/v_angle',
        ):

        self.gimbal_name = gimbal_name
        self.h_angle = None
        self.v_angle = None
        self.target_h_angle = None
        self.target_v_angle = None

        self.h_angle_range = [0, 350]
        self.v_angle_range = [-60, 20]

        self.pub_gimbal_h_control = rospy.Publisher(
            # name="/" + self.gimbal_name + "/gimbal/h_control",
            name = '/' + self.gimbal_name + gimbal_h_control_topic,
            data_class=Float32,
            queue_size=1,
        )

        self.pub_gimbal_v_control = rospy.Publisher(
            # name="/" + self.gimbal_name + "/gimbal/v_control",
            name = '/' + self.gimbal_name + gimbal_v_control_topic,
            data_class=Float32,
            queue_size=1,
        )

        self.sub_gimbal_h_angle = rospy.Subscriber(
            # name = '/' + self.gimbal_name + '/gimbal/h_angle',
            name = '/' + self.gimbal_name + gimbal_h_angle_topic,
            data_class=Float32,
            callback=self.gimbal_h_angle_cb,
        )

        self.sub_gimbal_v_angle = rospy.Subscriber(
            # name = '/' + self.gimbal_name + '/gimbal/v_angle',
            name = '/' + self.gimbal_name + gimbal_v_angle_topic,
            data_class=Float32,
            callback=self.gimbal_v_angle_cb,
        )

    def gimbal_h_angle_cb(self, msg):
        self.h_angle = msg.data

    def gimbal_v_angle_cb(self, msg):
        self.v_angle = msg.data

    def control_gimbal_h_angle(self, angle):
        self.pub_gimbal_h_control.publish(angle)

    def control_gimbal_v_angle(self, angle):
        self.pub_gimbal_v_control.publish(angle)

    def pan_tilt_move(self, pan_angle, tilt_angle):
        rate = rospy.Rate(0.5)
        if pan_angle is not None:
            if pan_angle >= self.h_angle_range[0] and pan_angle <= self.h_angle_range[1]:
                self.control_gimbal_h_angle(pan_angle)
                rate.sleep()
            else:
                rospy.logwarn("Pan angle out of range: " + str(pan_angle))
        if tilt_angle is not None:
            if tilt_angle >= self.v_angle_range[0] and tilt_angle <= self.v_angle_range[1]:
                self.control_gimbal_v_angle(tilt_angle)
                rate.sleep()
            else:
                rospy.logwarn("Tilt angle out of range: " + str(tilt_angle))

# class Gimbla_Server(object):

#     def __init__(
#             self,
#             control_topic = 'gimbla_target_angle',
#             hontal_angle_topic = 'gimbla_hontal_angle') -> None:
        
#         self.control_topic = control_topic
#         self.hontal_angle_topic = hontal_angle_topic

#         self.hontal_angle = 0
#         self.target_angle = None

#         self.control_pub = rospy.Publisher('/gimbla_target_angle', Float32, queue_size=2)
#         self.hontal_angle_sub = rospy.Subscriber('/gimbla_target_angle', Float32, self.hontal_angle_callback)

#     def hontal_angle_callback(self, msg):
#         self.hontal_angle = msg.data

#     def control_angle(self, angle):
#         self.control_pub.publish(angle)

# class Gimbla_client(object):

#     def __init__(
#             self,
#             control_topic = 'gimbla_target_angle',
#             hontal_angle_topic = 'gimbla_hontal_angle') -> None:
        
#         self.control_topic = control_topic
#         self.hontal_angle_topic = hontal_angle_topic

#         self.hontal_angle = 0
#         self.target_angle = None

#         self.control_pub = rospy.Publisher('/gimbla_target_angle', Float32, queue_size=2)
#         self.hontal_angle_sub = rospy.Subscriber('/gimbla_target_angle', Float32, self.hontal_angle_callback)

#     def hontal_angle_callback(self, msg):
#         self.hontal_angle = msg.data

#     def control_angle(self, angle):
#         self.control_pub.publish(angle)