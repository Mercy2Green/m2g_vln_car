### A higher node for the image and the gimbal to capture the image
### We using the fastlio to get the camera pose.

import rospy
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32



class Surrounding_image(object):

    def __init__(self, host, port, image_path):
        rospy.init_node('gimbal_commander', anonymous=True)
        self.host = host
        self.port = port
        self.image_path = image_path

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.images_dict = {}

        self.target_horizontal_angle = None
        self.target_vertical_angle = None
        self.current_horizontal_angle = None
        self.current_vertical_angle = None
        self.horizontal_target_angle_pub = rospy.Publisher('gimbla_control_horizontal_angle', Float32, queue_size=1)
        self.horizontal_angle_sub = rospy.Subscriber('gimbla_horizontal_angle', Float32, self.horizontal_angle_callback)

        self.pose_sub = rospy.Subscriber('camera_pose', Float32, self.pose_callback)


        self.server_thread = threading.Thread(target=self.start_server)
        #self.client_thread = threading.Thread(target=self.send_image)

    def horizontal_angle_callback(self, msg):
        self.current_horizontal_angle = msg.data
        #rospy.loginfo(f"Current Horizontal Angle: {self.current_horizontal_angle}")
    def vertical_angle_callback(self, msg):
        self.current_vertical_angle = msg.data

    def send_target_angles(self, horizontal_angle, vertical_angle):
        self.target_horizontal_angle = horizontal_angle
        self.target_vertical_angle = vertical_angle
        self.horizontal_target_angle_pub.publish(self.target_horizontal_angle)
        self.vertical_target_angle_pub.publish(self.target_vertical_angle)
        rospy.loginfo(f"Sent Target Horizontal Angle: {self.target_horizontal_angle}")

    def check_target_reached(self):
        if self.target_horizontal_angle is None:
            return False
        return (self.current_horizontal_angle >= (self.target_horizontal_angle - 0.1)and
                self.current_horizontal_angle <= (self.target_horizontal_angle + 0.1))

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)



