#### This file contains the meta move function that is used to move the robot to the goal location.

#### Asuming the robot only have linear velocity control and angular velocity control
#### It need the /cmd_vel (geometry_msgs::Twist)
#### The geometry_msgs::Twist has two fields: linear and angular
#### The linear field has two fields: x, y, z
#### The angular field has two fields: x, y, z

import rospy
import actionlib
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 0.5

class Ranger_mini_2_interface(object):

    def __init__(
            self,
            robot_name,
            use_move_base_action=False,
            pub_base_command_name="/cmd_vel",
            sub_base_odom_name="/odom",):

        self.robot_name = robot_name
        self.odom = Pose()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.base_odom_cb)
        self.use_move_base_action = use_move_base_action
        if (self.use_move_base_action):
            self.mb_client = actionlib.SimpleActionClient(
                ns="/" + self.robot_name + "/move_base",
                ActionSpec=MoveBaseAction,
            )
            self.mb_client.wait_for_server()

        self.pub_base_command = rospy.Publisher(
            # name="/cmd_vel",
            name=pub_base_command_name,
            data_class=Twist,
            queue_size=1,
        )

        self.pub_base_pose = rospy.Publisher(
            name="/" + self.robot_name + "/move_base_simple/goal",
            data_class=PoseStamped,
            queue_size=1,
        )

        self.sub_base_odom = rospy.Subscriber(
            # name="/odom",
            name=sub_base_odom_name,
            data_class=Odometry,
            callback=self.base_odom_cb,
        )

        rospy.sleep(0.5)
        print("Initialized Ranger_mini_2_interface!\n")

    ### @brief Move the base for a given amount of time
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @param duration - desired time [sec] that the robot should follow the specified speeds
    def move(self, x=0, yaw=0, duration=1.0):
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        # Publish Twist at 10 Hz for duration
        while (rospy.get_time() < (time_start + duration)):
            self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))
            r.sleep()
        # After the duration has passed, publish a zero Twist
        self.pub_base_command.publish(Twist())

    ### @brief Move the base to a given pose in a Map (Nav Stack must be enabled!)
    ### @param x - desired 'x' position [m] w.r.t. the map frame that the robot should achieve
    ### @param y - desired 'y' position [m] w.r.t. the map frame that the robot should achieve
    ### @param yaw - desired yaw [rad] w.r.t. the map frame that the robot should achieve
    ### @param wait - whether the function should wait until the base reaches its goal pose before
    ###        returning
    ### @return <bool> - whether the robot successfully reached its goal pose (only applies if
    ###        'wait' is `True`)
    ### @details - note that if 'wait' is `False`, the function will always return `True`.
    def move_to_pose(self, x, y, yaw, wait=False):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        if (wait and self.use_move_base_action):
            goal = MoveBaseGoal(target_pose)
            self.mb_client.send_goal(goal)
            self.mb_client.wait_for_result()
            if not (self.mb_client.get_state() == actionlib.GoalStatus.SUCCEEDED):
                rospy.logerr("Did not successfully reach goal...")
                return False
        else:
            self.pub_base_pose.publish(target_pose)
        return True
    
    ### @brief Commands a Twist message to the base
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @details - This method can be called repeatedly to move the robot if using a controller
    def command_velocity(self, x=0, yaw=0):
        self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))

    ### Get the 2D pose of the robot w.r.t. the robot 'odom' frame
    ### @return pose - list containing the [x, y, yaw] of the robot w.r.t. the odom frame
    def get_odom(self, odom:Pose):
        quat = (
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w
        )
        return [odom.position.x, odom.position.y, euler_from_quaternion(quat)[2]]

    ### @brief ROS Callback function to update the odometry of the robot
    ### @param msg - ROS Odometry message from the base
    def base_odom_cb(self, msg:Odometry):
        self.odom = msg.pose.pose



# if action_type == -1: # Point Navigation
#     locobot.base.move_to_pose(action[1].item(),action[2].item(),action[3].item(),wait=True)
#     locobot.base.mb_client.wait_for_result()

# else: # Atomic Actions
#     action_value = action[1].item()
#     if action_type==0: #forward
#         locobot.base.move(linear_speed, 0, move_time*action_value)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==1: #backward
#         locobot.base.move(-linear_speed, 0, move_time*action_value)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==2: #left
#         locobot.base.move(0, angular_speed, move_time*action_value)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==3 : #right
#         locobot.base.move(0, -angular_speed, move_time*action_value)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==4:  # camera down 30-degree
#         locobot.camera.pan_tilt_move(0, np.pi/6)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==5:  # camera up 30-degree
#         locobot.camera.pan_tilt_move(0, -np.pi/6)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==6:  # camera left 30-degree
#         locobot.camera.pan_tilt_move(np.pi/6, 0)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==7:  # camera right 30
#         locobot.camera.pan_tilt_move(-np.pi/6, 0)
#         #locobot.base.mb_client.wait_for_result()

#     elif action_type==8:  # camera go home
#         locobot.camera.pan_tilt_go_home() 
#         #locobot.base.mb_client.wait_for_result()
#     else:
#         print('Received an incorrect instruction!')





