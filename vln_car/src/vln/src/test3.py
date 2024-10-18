#! /usr/bin/env python

# Remote debugging setup
import debugpy
#debugpy.listen(5678)
#debugpy.wait_for_client()
# python3 -m debugpy --listen 5678 --wait-for-client vln_car/src/vln/src/test3.py

#from gimbal_node import GimblaNode
from control_angle import GimblaNode
import rospy
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading




class ImageClient:
    def __init__(self, rgb_sub_topic, depth_sub_topic, image_width, image_height):
        self.rgb_sub = message_filters.Subscriber(rgb_sub_topic, Image)
        self.depth_sub = message_filters.Subscriber(depth_sub_topic, Image)
        
        self.cv_bridge = CvBridge()
        self.sensor_lock = threading.Lock()
        
        self.rgb_img = None
        self.depth_img = None
        
        self.image_width = image_width
        self.image_height = image_height

        sync_msg = [self.rgb_sub, self.depth_sub]
        self.sensor_sync = message_filters.ApproximateTimeSynchronizer(
            sync_msg, queue_size=10, slop=0.2
        )
        self.sensor_sync.registerCallback(self._sync_callback)
        
        self.rgb_received = False
        self.depth_received = False


    def _sync_callback(self, rgb, depth):
        with self.sensor_lock:
            try:
                self.rgb_img = self.cv_bridge.imgmsg_to_cv2(rgb, "bgr8")
                self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
                
                self.rgb_received = True
                self.depth_received = True
            except CvBridgeError as e:
                rospy.logerr(e)

    def show_rgb(self):
        """
        This function displays the RGB image using OpenCV.
        """
        with self.sensor_lock:
            rgb = self.rgb_img
        if rgb is not None:
            resized_image = cv2.resize(rgb, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)
            cv2.imshow("RGB Image", resized_image)
            cv2.waitKey(1)

    def show_depth(self):
        """
        This function displays the depth image using OpenCV.
        """
        with self.sensor_lock:
            depth = self.depth_img
        if depth is not None:
            depth = depth # Convert to meters
            depth_mapped = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imshow("Depth Image", depth_mapped)
            cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('image_client')
    
    rgb_sub_topic = "/orbbec_camera/color/image_raw"
    depth_sub_topic = "/orbbec_camera/depth/image_raw"
    image_width = 1280
    image_height = 720

    client = ImageClient(rgb_sub_topic, depth_sub_topic, image_width, image_height)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if client.rgb_received:
            client.show_rgb()
        if client.depth_received:
            client.show_depth()
        rate.sleep()

