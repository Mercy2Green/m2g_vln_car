#!/usr/bin/env python

#### This file is used to move the ranger for testing purposes

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

import threading

# from ranger_mini_2_interface import Ranger_mini_2_interface
from VCBot import VCBot


def test_control_move():
    try:
        user_input = input("Please enter two numbers separated by a space, the first is the type, the second is the value: ")
        type_str, value_str = user_input.split()
        action_type = float(type_str)
        action_value = float(value_str)
        return action_type, action_value
    except ValueError:
        rospy.logerr("Invalid input. Please enter two valid numbers separated by a space.")
        return None
    except IndexError:
        rospy.logerr("Invalid input. Please enter exactly two numbers separated by a space.")
        return None

def move_rules(robot, action):

    linear_speed = 0.2
    angular_speed = np.pi / 3
    move_time = 0.1

    print(action)
    action_type = action[0]
    if action_type == -1:  # Point Navigation
        robot.base.move_to_pose(action[1].item(), action[2].item(), action[3].item(), wait=True)
        robot.base.mb_client.wait_for_result()

    else:  # Atomic Actions
        action_value = action[1]
        if action_type == 0:  # forward
            robot.base.move(linear_speed, 0, move_time * action_value)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 1:  # backward
            robot.base.move(-linear_speed, 0, move_time * action_value)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 2:  # left
            robot.base.move(0, angular_speed, move_time * action_value)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 3:  # right
            robot.base.move(0, -angular_speed, move_time * action_value)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 4:  # camera down 30-degree
            robot.gimbal.pan_tilt_move(0, np.pi / 6)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 5:  # camera up 30-degree
            robot.gimbal.pan_tilt_move(0, -np.pi / 6)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 6:  # camera left 30-degree
            robot.gimbal.pan_tilt_move(np.pi / 6, 0)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 7:  # camera right 30-degree
            robot.gimbal.pan_tilt_move(-np.pi / 6, 0)
            # robot.base.mb_client.wait_for_result()

        elif action_type == 8:  # camera go home
            robot.gimbal.pan_tilt_move(0,0)
            # robot.base.mb_client.wait_for_result()
        else:
            print('Received an incorrect instruction!')

def control_move(robot):
    while not rospy.is_shutdown():
        action = test_control_move()
        if action is not None:
            move_rules(robot, action)
        else:
            rospy.logerr("Invalid action. Please try again.")

def main():
    rospy.init_node('ranger_move_test_node', anonymous=True)

    robot = VCBot(
        robot_name="vcbot",
        gimbal_model="c40",
        gimbal_h_control_topic="/gimbla_h_target_angle",
        gimbal_v_control_topic="gimbal_v_target_angle",
        gimbal_h_angle_topic="/gimbla_horizontal_angle",
        gimbal_v_angle_topic="/gimbla_vertical_angle",
        base_model="ranger_mini_2",
        use_move_base_action=False,
        odom_model="livox",
        odom_offset=[-0.3, -0.1, 0, np.pi, 0, 0],
        odom_sub_topic="/fastlio_odom",
        odom_pub_topic="/odom",
        RGB_model="orbbec",
        RGB_WH=[1280, 720],
        RGB_FOV=[90, 65],
        RGB_sub_topic="/orbbec_camera/color/image_raw",
        Depth_model="orbbec",
        Depth_WH=[1280, 720],
        Depth_FOV=[90, 65],
        Depth_sub_topic="/orbbec_camera/depth/image_raw",
        Depth_upscale=1.0,
        Depth_downscale=1000,)
    
    rospy.loginfo("Ranger move test node started.")

    control_thread = threading.Thread(target=control_move, args=(robot,))
    control_thread.daemon = True
    control_thread.start()

    while not rospy.is_shutdown():
        rospy.sleep(1)
    
    rospy.loginfo("Ranger move test node shutting down.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass