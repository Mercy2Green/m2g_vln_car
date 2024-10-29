#!/usr/bin/env python
import numpy as np

L2C_TRANSFORM = np.array(
    [   [-0.00859563,-0.999714,-0.0223111,0.01927],
        [-0.0428589,0.0227041,-1.00387,0.699481],
        [0.999075,-0.00764551,-0.0423244,0.0611374],
        [0,0,0,1]])

C2G_TRANSFORM = np.array(
    [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -0.06],
        [0, 0, 0, 1]
    ])
    
G2L_TRANSFORM = np.array(
    [
       [-0.00859563,-0.999714,-0.0223111,0.01927],
       [-0.0428589,0.0227041,-1.00387,0.699481],
       [0.999075,-0.00764551,-0.0423244,0],
       [0,0,0,1] 
    ])

CAMERA_INTRINSIC =  np.array([  [610.6340942382812, 0.0, 635.8567504882812], 
                                [0.0, 610.8860473632812, 355.1409912109375], 
                                [0.0, 0.0, 1.0]])

TARGET_ANGLE_LIST = np.array([0,30,60,90,120,150,180,210,240,270,300,330])

DEPTH_SUB_TOPIC = '/orbbec_camera/depth/image_raw'
RGB_SUB_TOPIC = '/orbbec_camera/color/image_raw'
ODOM_SUB_TOPIC = '/fastlio_odom'