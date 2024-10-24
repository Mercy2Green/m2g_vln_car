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
        self.images_dict = {}

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
                if key == 0:
                    conn.sendall(b'send_image')
                    data = b''
                    while True:
                        packet = conn.recv(409600)
                        if not packet:
                            print("No more data")
                            break
                        data += packet
                    images_dict = pickle.loads(data)
                    for key, image_list in images_dict.items():
                        for i in range(len(image_list)):
                            save_image = cv2.imdecode(np.frombuffer(image_list[i], np.uint8), cv2.IMREAD_COLOR)
                            save_name = f'images_1/{key}_{i*30}.png'
                            cv2.imwrite(save_name, save_image)
                    # if cv2.waitKey(0) & 0xFF == ord('a'):
                    #     self.close_all_windows()
                        # file_name = f'images_0/{key}_0.png'
                        # cv2.imwrite(file_name, image)
                else:
                    conn.sendall(b'Rotate_once')
            # angle_dict = {}
            # for key, angle_dict in self.images_dict.items():
            #     for angle_key, image_bytes in angle_dict.items():
            #         image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
            #         #labeled_image = self.label_image(image, f'{key}_angle: {angle_key}')
            #         cv2.imshow(f'{key}_{angle_key}', image)
            #         file_name = f'images/{key}_{angle_key}.png'
            #         cv2.imwrite(file_name, image)
            #         #self.save_images(key, angle_key, image)
            #         if cv2.waitKey(0) & 0xFF == ord('a'):
            #             self.close_all_windows()
                


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
            #while not rospy.is_shutdown():
            # while True:
            #     command = s.recv(1024)
            #     if not command:
            #         break
            command = s.recv(1024)
            if command == b'Rotate_once':
                for angle in range(0, 360, 30):
                #     self.images_dict[angle] = {}
                    self.target_horizontal_angle = angle
                    if angle > 0:
                        self.current_horizontal_angle = angle - 1
                    else:
                        self.current_horizontal_angle = 1
                    self.send_target_angles(angle, 0)
                    # #angle = float(input(f"Enter the horizontal angle to control the gimbal: "))
                    while True:
                        if self.check_target_reached()== False:
                            #conn.sendall(b'send_image')
                            self.send_target_angles(angle, 0)
                            time.sleep(1)
                        else:
                            rospy.loginfo(f"Success reach Horizontal Angle: {self.target_horizontal_angle}")
                            rospy.loginfo(f"Current Horizontal Angle: {self.current_horizontal_angle}")
                            
                            rgb_image, depth_image = self.capture_images()
                            rgb_name = f'images_0/rgb_{angle}.png'
                            depth_name = f'images_0/depth_{angle}.png'
                            cv2.imwrite(rgb_name, rgb_image)
                            cv2.imwrite(depth_name, depth_image)
                            time.sleep(1)
                            break
            # while True:
            #     command1 = s.recv(1024)
            #     if not command1:
            #         break
            if command == b'send_image':
            #if command == b'send_image':       
                # rgb_image, depth_image = self.capture_images()
                # #image = cv2.imread(image_path)
                # images_dict = {}
                # _, rgb_bytes = cv2.imencode('.png', rgb_image)
                # _, depth_bytes = cv2.imencode('.png', depth_image)
                # images_dict['rgb'] = rgb_bytes.tobytes()
                # images_dict['depth'] = depth_bytes.tobytes()
                # data = pickle.dumps(images_dict)
                # s.sendall(data)
                images_dict = {}
                rgb_list = []
                depth_list = []
                for angle in range(0, 360, 30):
                    rgb_name = f'images_0/rgb_{angle}.png'
                    depth_name = f'images_0/depth_{angle}.png'
                    rgb_image = cv2.imread(rgb_name)
                    depth_image = cv2.imread(depth_name)
                    _, rgb_bytes = cv2.imencode('.png', rgb_image)
                    _, depth_bytes = cv2.imencode('.png', depth_image)
                    rgb_list.append(rgb_bytes.tobytes())
                    depth_list.append(depth_bytes.tobytes())
                images_dict['rgb'] = rgb_list
                images_dict['depth'] = depth_list
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
