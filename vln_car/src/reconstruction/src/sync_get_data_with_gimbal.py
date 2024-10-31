#!/usr/bin/env python
# Using this code to get the sync data from the camera and the lidar

import debugpy
debugpy.listen(2457)

from configparser import ConfigParser
import os
import random
import socket
import string
import threading
import pickle
import cv2
import copy
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

import message_filters

from param import L2C_TRANSFORM, RGB_SUB_TOPIC, DEPTH_SUB_TOPIC, ODOM_SUB_TOPIC, CAMERA_INTRINSIC, G2L_TRANSFORM, C2G_TRANSFORM, TARGET_ANGLE_LIST
from gimbal_interface import Gimbal_interface

class SyncGetData:

    def __init__(self):

        rospy.init_node('sync_get_data', anonymous=True)

        self.cv_bridge = CvBridge()
        self.sensor_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.lpose = None

        self.gimbal = Gimbal_interface()

        self.rate = rospy.Rate(1) # 1hz

        self.rate_gimbal = rospy.Rate(10)

        self.past_h_angle = None

        self.rgb_sub = message_filters.Subscriber(RGB_SUB_TOPIC, Image)
        self.depth_sub = message_filters.Subscriber(DEPTH_SUB_TOPIC, Image)
        self.odom_sub = message_filters.Subscriber(ODOM_SUB_TOPIC, Odometry)

        sync_msg = [self.rgb_sub, self.depth_sub, self.odom_sub]
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            sync_msg, queue_size=100, slop=0.05
        )
        # self.sensor_sync = message_filters.TimeSynchronizer(sync_msg, queue_size = 100)

        self.sensor_sync.registerCallback(self._sync_callback)

        while not rospy.is_shutdown():
            if self.rgb_img is not None and self.depth_img is not None and self.odom is not None:
                # if self.gimbal.h_angle is not None:
                #     break
                # else:
                #     print("Waiting for gimbal data...")
                #     time.sleep(1)
                break
            else:
                print("Waiting for sensor data...")
                time.sleep(1)

    def check_gimbal(self, angle, target_angle, threshold=0.05):
        if angle is not None:
            if abs(angle - target_angle) < threshold:
                return True
            else :
                return False
        else:
            return False

    def _sync_callback(self, rgb, depth, odom):

            with self.sensor_lock:
                try:
                    _odom = odom.pose.pose
                    self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
                    self.rgb_img = self.rgb_img[:, :, ::-1]
                    self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
                    self.odom = _odom
                    self.lpose = self.odom_2_pose(_odom)
                    # self.cpose = self.lpose_2_cpose(self.lpose, L2C_TRANSFORM)
                except CvBridgeError as e:
                    rospy.logerr(e)

    def odom_2_pose(self, odom):
        x,y,z = odom.position.x, odom.position.y, odom.position.z
        quat = (
            odom.orientation.x,
            odom.orientation.y,
            odom.orientation.z,
            odom.orientation.w
        )
        rotation_matrix = np.array([[1-2*(quat[1]**2+quat[2]**2), 2*(quat[0]*quat[1]-quat[2]*quat[3]), 2*(quat[0]*quat[2]+quat[1]*quat[3])],
                                     [2*(quat[0]*quat[1]+quat[2]*quat[3]), 1-2*(quat[0]**2+quat[2]**2), 2*(quat[1]*quat[2]-quat[0]*quat[3])],
                                     [2*(quat[0]*quat[2]-quat[1]*quat[3]), 2*(quat[1]*quat[2]+quat[0]*quat[3]), 1-2*(quat[0]**2+quat[1]**2)]])
        position = np.array([x,y,z])

        pose= np.eye(4)
        pose[:3,:3] = rotation_matrix
        pose[:3,3] = position

        return pose
        
    def lpose_2_cpose(self, lpose, L2C_TRANSFORM):
        cpose = np.dot(L2C_TRANSFORM, lpose)
        return cpose

    def get_data(self):
        with self.sensor_lock:
            rgb = copy.deepcopy(self.rgb_img)
            depth = copy.deepcopy(self.depth_img)
            odom = copy.deepcopy(self.odom)
            lpose = copy.deepcopy(self.lpose)
            # cpose = copy.deepcopy(self.cpose)
        return rgb, depth, odom, lpose
    
    def save_data_gimbal(self, save_root_path, target_angles_list):

        rate = rospy.Rate(5.9) # 1.3hz

        images_list = []
        images_name_list = []
        depths_list = []
        depths_name_list = []
        odoms_list = []
        lposes_list = []
        positions_list = []
        angles_list = []

        camera_intrinsics = CAMERA_INTRINSIC
        
        config = ConfigParser()

        # Randomly generate a string for the name of config[i]
        vp_name = ''.join(random.choices(string.ascii_uppercase + string.digits, k=6))
        for idx, angle in enumerate(target_angles_list):
            while self.check_gimbal(self.gimbal.h_angle, angle) is False:
                cur_angle = self.gimbal.h_angle
                self.gimbal.pan_tilt_move(angle, None)
                print(f"Wait for the gimbal to move to the {cur_angle}/{angle} angle")
                rate.sleep()

            # self.wait_for_gimbal(angle, rate_gimbal)
            if self.check_gimbal(self.gimbal.h_angle, angle):
                print(f"Get the {idx+1}th data")
                bgr, depth, odom, lpose = self.get_data()
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                images_list.append(rgb)
                images_name_list.append(f"{vp_name}_i_{idx}.png")
                depths_list.append(depth)
                depths_name_list.append(f"{vp_name}_d_{idx}.png")
                odoms_list.append(odom)
                x, y, z = odom.position.x, odom.position.y, odom.position.z
                positions_list.append(np.array([x, y, z])) 
                lposes_list.append(lpose)
                angles_list.append(angle)
                print(f"Saved the {idx+1}/{len(target_angles_list)}th data")
            else:
                print(f"Error in getting the {idx+1}th data")

        self.gimbal.pan_tilt_move(0, None)

        os.makedirs(save_root_path, exist_ok=True)
        os.makedirs(f"{save_root_path}/camera_parameter", exist_ok=True)
        os.makedirs(f"{save_root_path}/color_image", exist_ok=True)
        os.makedirs(f"{save_root_path}/depth_image", exist_ok=True)

        for i in range(len(images_list)):

            cv2.imwrite(f"{save_root_path}/color_image/{images_name_list[i]}", images_list[i])
            cv2.imwrite(f"{save_root_path}/depth_image/{depths_name_list[i]}", depths_list[i])

            config[vp_name] = {
                'rgb_name': images_name_list,
                'depth_name': depths_name_list,
                'lpose': lposes_list,
                'heading': angles_list,
                'position': positions_list,
                'camera_intrinsics': camera_intrinsics.tolist(),  # convert numpy array to list
                'l2c_transform': L2C_TRANSFORM.tolist(),
                'c2g_transform': C2G_TRANSFORM.tolist(),
                'g2l_transform': G2L_TRANSFORM.tolist(),
                'target_angle_list': target_angles_list.tolist(),
                'heading_direction': 'CounterClockwise',
                }

            print(f"Saved {i+1}/{len(images_list)}th data")
            
        with open(f"{save_root_path}/camera_parameter/camera_parameter.conf", 'a') as f:
            config.write(f)
        print("#######################*******Finish saving data********#######################")

        # back_rate = rospy.Rate(0.1)
        while self.check_gimbal(self.gimbal.h_angle, 0) is False:
            self.gimbal.pan_tilt_move(0, None)
            print(f"Wait for the gimbal to go back to the {self.gimbal.h_angle}/0 degree")
            rate.sleep()
        print("*******Gimbal ready********")
            
    def wait_for_gimbal(self, target_angle, control_rate):
        while self.check_gimbal(self.gimbal.h_angle, target_angle) is False:
            self.gimbal.pan_tilt_move(target_angle, None)
            print(f"Wait for the gimbal to go to the {target_angle} degree")
            control_rate.sleep()

    def gimbal_rotate(self, target_h_angle = None, target_v_angle = None, control_rate = 1):
        while True:
            self.gimbal.pan_tilt_move(target_h_angle, target_v_angle)
            time.sleep(control_rate)


    def start(self):

        exp_name = None
        while not rospy.is_shutdown():

            start_flag = input("Do you want to start the data collection? (y/n): ")
            if start_flag == 'y':
                if exp_name is None:
                    exp_name = input("Enter the experiment name: ")
                print("Start the data collection")
                self.save_data_gimbal(f"/home/uav/m2g_vln_car/datasets/gimbal/{exp_name}", TARGET_ANGLE_LIST)       
            else:
                print('Enter "y" to start the data collection')

            time.sleep(0.5)

    def test_start(self):
        while not rospy.is_shutdown():

            self.save_data_gimbal(f"/home/uav/m2g_vln_car/datasets/gimbal/test", TARGET_ANGLE_LIST)

            time.sleep(0.5)

if __name__ == "__main__":
    try:
        # SyncGetData().start()
        SyncGetData().test_start()
    except rospy.ROSInterruptException:
        pass

