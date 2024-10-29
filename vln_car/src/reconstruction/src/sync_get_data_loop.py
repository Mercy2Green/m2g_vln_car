#!/usr/bin/env python
# Using this code to get the sync data from the camera and the lidar

from configparser import ConfigParser
import os
import socket
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

from param import L2C_TRANSFORM, RGB_SUB_TOPIC, DEPTH_SUB_TOPIC, ODOM_SUB_TOPIC, CAMERA_INTRINSIC

class SyncGetData:

    def __init__(self):

        rospy.init_node('sync_get_data', anonymous=True)

        self.cv_bridge = CvBridge()
        self.sensor_lock = threading.RLock()
        self.rgb_img = None
        self.depth_img = None
        self.lpose = None

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
            # if self.rgb_img is not None and self.depth_img is not None:
                break
            print("Waiting for sensor data...")
            time.sleep(1)

    def check_gimbal(self, angle, target_angle):

        if target_angle is not None:
            if abs(angle - target_angle) < 0.02:
                return True
            else:
                return False
        else:
            return

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
    
    def save_data(self, save_root_path, idx_max=5):

        idx = 0
        rate = rospy.Rate(1) # 1hz

        images_list = []
        images_name_list = []
        depths_list = []
        depths_name_list = []
        odoms_list = []
        lposes_list = []
        camera_intrinsics = CAMERA_INTRINSIC
        
        config = ConfigParser()

        while not rospy.is_shutdown():
                
            bgr, depth, odom, lpose = self.get_data()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            images_list.append(rgb)
            images_name_list.append(f"i_{idx}.png")
            depths_list.append(depth)
            depths_name_list.append(f"d_{idx}.png")
            odoms_list.append(odom)
            lposes_list.append(lpose)

            
            print(f"#########*******Get {idx+1}/{idx_max}th data*******##############")
            idx += 1

            rate.sleep()

            if idx == idx_max:
                break

        os.makedirs(save_root_path, exist_ok=True)
        os.makedirs(f"{save_root_path}/camera_parameter", exist_ok=True)
        os.makedirs(f"{save_root_path}/color_image", exist_ok=True)
        os.makedirs(f"{save_root_path}/depth_image", exist_ok=True)
        with open(f"{save_root_path}/camera_parameter/camera_parameter.conf", 'w') as f:
            print("Create camera_parameter.conf file")

        for i in range(len(images_list)):

            cv2.imwrite(f"{save_root_path}/color_image/{images_name_list[i]}", images_list[i])
            cv2.imwrite(f"{save_root_path}/depth_image/{depths_name_list[i]}", depths_list[i])

            config[i] = {
                'idx': i,
                'rgb_name': images_name_list[i],
                'depth_name': depths_name_list[i],
                'lpose': lposes_list[i],
                # 'odoms': odoms_list[i],
                'camera_intrinsics': camera_intrinsics.tolist(),  # convert numpy array to list
                'l2c_transform': L2C_TRANSFORM.tolist()
                }

            print(f"Saved {i+1}/{idx_max}th data")
            
        with open(f"{save_root_path}/camera_parameter/camera_parameter.conf", 'a') as f:
            config.write(f)

            

    def start(self):

        while not rospy.is_shutdown():

            start_flag = input("Do you want to start the data collection? (y/n): ")
            if start_flag == 'y':
                seconds = input("Enter the time duration for data collection (s): ")
                # self.save_data(f"/home/uav/m2g_vln_car/datasets/{exp_name}", idx_max=10)
                try:
                    seconds = int(seconds)
                    print("Start the data collection")
                    date = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
                    exp_name = f"{date}_exp"
                    self.save_data(f"/home/uav/m2g_vln_car/datasets/loop/{exp_name}", idx_max=seconds)
                except:
                    print("Error in saving data")
                print("#######################*******Finish saving data********#######################")
            else:
                print('Enter "y" to start the data collection')

            time.sleep(0.5)

        # # date = time.strftime("%Y-%m-%d-%H-%M", time.localtime())
        # # exp_name = f"{date}_exp"
        # # self.save_data(f"/home/uav/m2g_vln_car/datasets/{exp_name}", idx_max=100)

        # while not rospy.is_shutdown():
            
        #     time.sleep(0.2)

        # rospy.spin()



if __name__ == "__main__":
    try:
        SyncGetData().start()
    except rospy.ROSInterruptException:
        pass
