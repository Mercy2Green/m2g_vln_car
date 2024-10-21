#! /usr/bin/env python
import socket
import threading
import pickle
import cv2
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
class GimbalCommander:
    def __init__(self, host='10.12.125.172', port=5003, image_path='images/rgb_0.png'):
        rospy.init_node('gimbal_commander', anonymous=True)
        self.host = host
        self.port = port
        self.image_path = image_path

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        self.target_horizontal_angle = None
        self.target_vertical_angle = None
        self.current_horizontal_angle = None
        self.current_vertical_angle = None
        self.horizontal_target_angle_pub = rospy.Publisher('gimbla_control_horizontal_angle', Float32, queue_size=1)
        self.vertical_target_angle_pub = rospy.Publisher('gimbla_control_vertical_angle', Float32, queue_size=1)

        # Subscribers to receive current angles
        self.horizontal_angle_sub = rospy.Subscriber('gimbla_horizontal_angle', Float32, self.horizontal_angle_callback)
        self.vertical_angle_sub = rospy.Subscriber('gimbla_vertical_angle', Float32, self.vertical_angle_callback)
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
        #rospy.loginfo(f"Sent Target Horizontal Angle: {self.target_horizontal_angle}")

    def check_target_reached(self):
        if self.target_horizontal_angle is None or self.target_vertical_angle is None:
            return False
        return (self.current_horizontal_angle == self.target_horizontal_angle and
                self.current_vertical_angle == self.target_vertical_angle)

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

    def capture_images(self):
        #rospy.init_node('image_capture_node', anonymous=True)
        rospy.Subscriber('/orbbec_camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/orbbec_camera/depth/image_raw', Image, self.depth_callback)

        while self.rgb_image is None or self.depth_image is None:
            rospy.sleep(0.1)

        return self.rgb_image, self.depth_image
    def label_image(self, image, label):
        """
        Label the image with the given label.
        """
        labeled_image = image.copy()
        cv2.putText(labeled_image, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        return labeled_image
    
    def start_server(self):

        def handle_client(conn, addr):
            print(f"Connected by {addr}")
            while not rospy.is_shutdown():
                key = float(input(f"Enter the horizontal angle to control the gimbal: "))
                self.send_target_angles(key, 0)
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
                    #while not rospy.is_shutdown():


        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen()
            print(f"Server listening on {self.host}:{self.port}")
            conn, addr = s.accept()
            with conn:
                handle_client(conn, addr)

    def send_image(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.host, self.port))
            command = s.recv(1024)
            if command == b'send_image':
                rgb_image, depth_image = self.capture_images()
                #image = cv2.imread(image_path)
                images_dict = {}
                _, rgb_bytes = cv2.imencode('.png', rgb_image)
                _, depth_bytes = cv2.imencode('.png', depth_image)
                images_dict['rgb'] = rgb_bytes.tobytes()
                images_dict['depth'] = depth_bytes.tobytes()
                data = pickle.dumps(images_dict)
                s.sendall(data)

    def start(self):
        self.server_thread.start()
        time.sleep(0.1)  # Give the server a moment to start
        #self.client_thread.start()
        self.send_image()
        #self.server_thread.join()
        #self.client_thread.join()

if __name__ == "__main__":
    try:
        commander = GimbalCommander()
        commander.start()
    except rospy.ROSInterruptException:
        pass
