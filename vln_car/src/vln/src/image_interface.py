### This is for RGB, depth, and other iamge sensors

import rospy
import tf
import argparse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import numpy as np
from sensor_msgs.msg import Image
import cv2

class Image_interface(object):

    def __init__(
        self,
        image_model = 'RGB',
        image_WH = [640, 480], # width, height
        image_FOV = [60, 60], # HFOV, VFOV
        image_sub_topic = "/camera/color/image_raw",
        pixel_upscale = 1.0,
        pixel_downscale = 1.0,
        ):


        self.image_WH = image_WH
        self.image_FOV = image_FOV

        self.image_sub_topic = image_sub_topic
        self.pixel_upscale = pixel_upscale
        self.pixel_downscale = pixel_downscale
        self.image_model = image_model

    def get_sub_topic(self):
        return self.image_sub_topic

    def resize_image(self, image, width, height):
        return cv2.resize(image, (width, height))
    
    def crop_image_from_center(self, image, x, y, width, height):
        return image[y - height // 2:y + height // 2, x - width // 2:x + width // 2]
    
    def crop_image_from_top_left(self, image, x, y, width, height):
        return image[y:y + height, x:x + width]
    
    def upscale_image_pixel(self, image, scale):
        # Each pixel multiplied by scale
        return cv2.multiply(image, scale)
    
    def downscale_image_pixel(self, image, scale):
        # Downscale image
        return cv2.multiply(image, 1.0 / scale)
    
    def calculate_fx_fy_cx_cy(self, width, height, hfov, vfov):
        fx = width / (2 * np.tan(hfov * np.pi / 360))
        fy = height / (2 * np.tan(vfov * np.pi / 360))
        cx = width / 2
        cy = height / 2
        return fx, fy, cx, cy
    
    def calculate_camera_intrinsics_matrix(self, width, height, hfov, vfov):
        fx, fy, cx, cy = self.calculate_fx_fy_cx_cy(width, height, hfov, vfov)
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])