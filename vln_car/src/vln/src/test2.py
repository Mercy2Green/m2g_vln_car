#! /usr/bin/env python
import socket
import threading
import cv2
import numpy as np
import time
import pickle
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

class GimbalCommander:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gimbal_commander', anonymous=True)

        # Publishers to send target angles
        self.horizontal_target_angle_pub = rospy.Publisher('gimbla_control_horizontal_angle', Float32, queue_size=1)
        self.vertical_target_angle_pub = rospy.Publisher('gimbla_control_vertical_angle', Float32, queue_size=1)

        # Subscribers to receive current angles
        self.horizontal_angle_sub = rospy.Subscriber('gimbla_horizontal_angle', Float32, self.horizontal_angle_callback)
        self.vertical_angle_sub = rospy.Subscriber('gimbla_vertical_angle', Float32, self.vertical_angle_callback)

        self.target_horizontal_angle = None
        self.target_vertical_angle = None
        self.current_horizontal_angle = None
        self.current_vertical_angle = None

    def horizontal_angle_callback(self, msg):
        self.current_horizontal_angle = msg.data
        #rospy.loginfo(f"Current Horizontal Angle: {self.current_horizontal_angle}")

    def vertical_angle_callback(self, msg):
        self.current_vertical_angle = msg.data
        #rospy.loginfo(f"Current Vertical Angle: {self.current_vertical_angle}")

    def send_target_angles(self, horizontal_angle, vertical_angle):
        self.target_horizontal_angle = horizontal_angle
        self.target_vertical_angle = vertical_angle
        self.horizontal_target_angle_pub.publish(self.target_horizontal_angle)
        self.vertical_target_angle_pub.publish(self.target_vertical_angle)
        rospy.loginfo(f"Sent Target Horizontal Angle: {self.target_horizontal_angle}")
        #rospy.loginfo(f"Sent Target Vertical Angle: {self.target_vertical_angle}")

    def check_target_reached(self):
        if self.target_horizontal_angle is None or self.target_vertical_angle is None:
            return False
        return (self.current_horizontal_angle == self.target_horizontal_angle and
                self.current_vertical_angle == self.target_vertical_angle)
bridge = CvBridge()
# Server function
def start_server(host='10.12.125.172', port=5000):
    def handle_client(conn, addr):
        print(f"Connected by {addr}")
        key = float(input(f"Enter the horizontal angle to control the gimbal: "))
        if key == 0.0:
            conn.sendall(b'send_image')
        data = b''
        while True:
            packet = conn.recv(4096)
            if not packet:
                print("No more data")
                break
            data += packet
            #print(data)
        # print(data)  # Print the received data for debugging
        images_dict = pickle.loads(data)
        for key, image_bytes in images_dict.items():
            image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow(key, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        conn, addr = s.accept()
        with conn:
            handle_client(conn, addr)

# Client function
def send_image(host='10.12.125.172', port=5000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        command = s.recv(1024)
        if command == b'send_image':
            rgb_image, depth_image = capture_images()
            #image = cv2.imread(image_path)
            images_dict = {}
            _, rgb_bytes = cv2.imencode('.png', rgb_image)
            _, depth_bytes = cv2.imencode('.png', depth_image)
            images_dict['rgb'] = rgb_bytes.tobytes()
            images_dict['depth'] = depth_bytes.tobytes()
            data = pickle.dumps(images_dict)
            s.sendall(data)
            #print('000000')
def capture_images():
    rgb_image = None
    depth_image = None

    def rgb_callback(msg):
        nonlocal rgb_image
        try:
            rgb_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(msg):
        nonlocal depth_image
        try:
            depth_image = bridge.imgmsg_to_cv2(msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

    rospy.init_node('image_capture_node', anonymous=True)
    rospy.Subscriber('/orbbec_camera/color/image_raw', Image, rgb_callback)
    rospy.Subscriber('/orbbec_camera/depth/image_raw', Image, depth_callback)

    while rgb_image is None or depth_image is None:
        rospy.sleep(0.1)

    return rgb_image, depth_image
if __name__ == "__main__":
    # Start the server in a separate thread
    server_thread = threading.Thread(target=start_server)
    server_thread.start()  # Do not set as daemon
    commander = GimbalCommander()
    # Give the server a moment to start
    time.sleep(1)

    # Start the client
    # key = float(input(f"Enter the horizontal angle to control the gimbal: "))
    # if key == 0.0:
    send_image()

    # Wait for the server thread to finish
    server_thread.join()