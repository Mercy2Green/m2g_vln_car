#! /usr/bin/env python

# import debugpy
# debugpy.listen(5678)
# debugpy.wait_for_client()


import os
# from interbotix_xs_modules.locobot import InterbotixLocobotXS
from ranger_mini_2_interface import Ranger_mini_2_interface
from VCBot import VCBot

import socket
import copy
import threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import numpy as np
import rospy
from abc import ABCMeta, abstractmethod
import message_filters
import cv2

import time
import paramiko
import rospy
from std_srvs.srv import *
import pickle

from std_msgs.msg import Float32, Float32MultiArray

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# This script commands arbitrary positions to the pan-tilt servos when using Time-Based-Profile for its Drive Mode
# When operating motors in 'position' control mode, Time-Based-Profile allows you to easily set the duration of a particular movement
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_base'
# Then change to this directory and type 'python pan_tilt_control.py'
    
linear_speed = 0.2
angular_speed = np.pi/3
move_time = 0.1
turn_time = 0.1

height=720  
width=1280
channel=3

Server_IP = '10.120.17.98'
Robot_IP = '10.12.125.172'

class TCPClient:
    def __init__(self,host:str,port:int):
        self.client=socket.socket()
        start_state = False
        while start_state==False:
            try:
                self.client.connect((host,port))
                start_state = True
                print('Client Start.')
            except:
                time.sleep(1)

    def send_data(self,data:bytes):
        data = pickle.dumps(data)
        self.client.send(pickle.dumps(len(data)).ljust(64))
        self.client.sendall(data)

    def close(self):
        self.client.close()

class TCPServer:
    def __init__(self,host:str,port:int):
        self.server=socket.socket()
        start_state = False
        while start_state==False:
            try:
                self.server.bind((host,port))
                self.server.listen(1)
                start_state = True
                print('Server Start.')
            except:
                time.sleep(1)
        self.client_socket, self.clientAddr = self.server.accept()

    def recv_data(self):
        while True:
            try:
                data_len = self.client_socket.recv(64)
                break
            except:
                time.sleep(1)

        data_len = pickle.loads(data_len)
        buffer = b"" 
        while True:
            received_data = self.client_socket.recv(512)
            buffer = buffer + received_data 
            if len(buffer) == data_len: 
                break
        data = pickle.loads(buffer) 
        return data

    def close(self):
        self.server.close()
class VLN_client(object):

    __metaclass__ = ABCMeta

    def __init__(
        self,
        rgb_sub_topic,
        depth_sub_topic,
        odom_sub_topic,
        image_width,
        image_height,
        depth_downscale,):

        self.image_width = image_width
        self.image_height = image_height
        self.depth_downscale = depth_downscale

        self.cv_bridge = CvBridge()
        self.sensor_lock = threading.RLock()

        self.rgb_img = None
        self.depth_img = None
        self.odom = None

        self.rgb_sub = message_filters.Subscriber(rgb_sub_topic, Image)
        self.depth_sub = message_filters.Subscriber(depth_sub_topic, Image)
        self.odom_sub = message_filters.Subscriber(odom_sub_topic, Odometry)

        sync_msg = [self.rgb_sub, self.depth_sub, self.odom_sub]
        # sync_msg = [self.rgb_sub, self.depth_sub]
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            sync_msg, queue_size=100, slop=0.2
        )
        self.sensor_sync.registerCallback(self._sync_callback)
        while not rospy.is_shutdown():
            if self.rgb_img is not None and self.depth_img is not None and self.odom is not None:
            # if self.rgb_img is not None and self.depth_img is not None:
                break
            print("Waiting for sensor data...")
            time.sleep(1)

    def _sync_callback(self, rgb, depth, odom):
    # def _sync_callback(self, rgb, depth):
        with self.sensor_lock:
            try:
                self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
                self.rgb_img = self.rgb_img[:, :, ::-1]
                self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")

                self.odom = odom


            except CvBridgeError as e:
                rospy.logerr(e)

    def get_rgb(self):
        """
        This function returns the RGB image perceived by the camera.
        :rtype: np.ndarray or None
        """
        with self.sensor_lock:
            rgb = copy.deepcopy(self.rgb_img)
        if rgb is not None:
            resized_image = cv2.resize(rgb, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)
            return resized_image

    def get_depth(self):
        """
        This function returns the depth image perceived by the camera.
        
        The depth image is in meters.
        
        :rtype: np.ndarray or None
        """
        with self.sensor_lock:
            depth = copy.deepcopy(self.depth_img)
        if depth is not None:
            depth = depth / 1000.
            ### depth threshold 0.2m - 2m
            depth = depth.reshape(-1)
            depth = depth.reshape(height,width)
            depth_mapped = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.3), cv2.COLORMAP_JET)
            return depth, depth_mapped
    
    def get_odom(self):
        with self.sensor_lock:
            odom = self.odom
        if odom is not None:
            return odom.pose.pose

def main():

    rospy.init_node('vln_client', anonymous=True)

    start_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    
    rgb_client=TCPClient(Server_IP,5001)
    depth_client=TCPClient(Server_IP,5002)
    location_client=TCPClient(Server_IP,5003)
    action_server = TCPServer(Robot_IP,5000)

    robot = VCBot(
        robot_name="vcbot",
        gimbal_model="c40",
        gimbal_h_control_topic="/gimbla_h_target_angle",
        gimbal_v_control_topic="gimbal_v_target_angle",
        gimbal_h_angle_topic="/gimbla_horizontal_angle",
        gimbal_v_angle_topic="/gimbla_vertical_angle",
        base_model="ranger_mini_2",
        use_move_base_action=False,
        odom_model=None,
        # odom_offset=[-0.3, -0.1, 0, np.pi, 0, 0],
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

    vln_client = VLN_client(
        rgb_sub_topic=robot.RGB_sub_topic,
        depth_sub_topic=robot.Depth_sub_topic,
        # odom_sub_topic=robot.odom_pub_topic,
        odom_sub_topic="/fastlio_odom",
        image_width=robot.RGB_WH[0],
        image_height=robot.RGB_WH[1],
        depth_downscale=robot.Depth_downscale,
        )

    while not rospy.is_shutdown():
        print('Waiting for action...')

        #### rgbd image 
        rgb = vln_client.get_rgb()
        depth, depth_mapped = vln_client.get_depth()
        
        # location = robot.base.get_odom(robot.base.odom)
        location = robot.base.get_odom(vln_client.get_odom())

        print("The location is :", location)
        rgb_client.send_data(rgb)
        depth_client.send_data(depth)
        location_client.send_data(location)
        action = action_server.recv_data()
        print(action)
        #locobot.camera.pan_tilt_go_home() 

        action_type = action[0].item()
        if action_type == -1:  # Point Navigation
            robot.base.move_to_pose(action[1].item(), action[2].item(), action[3].item(), wait=True)
            robot.base.mb_client.wait_for_result()

        else:  # Atomic Actions
            action_value = action[1].item()
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

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass