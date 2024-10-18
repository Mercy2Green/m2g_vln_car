#! /usr/bin/env python
import socket
import threading
import time
import pickle
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def start_server(host='10.12.125.172', port=5000):
    def handle_client(conn, addr):
        print(f"Connected by {addr}")
        while True:
            key = float(input(f"Enter the horizontal angle to control the gimbal: "))
            #key = cv2.waitKey(1) & 0xFF
            if key == 0.0:
                conn.sendall(b'capture')
            elif key == 1.0:
                conn.sendall(b'send')
                #time.sleep(0.1)
                data = b''
                while True:
                    packet = conn.recv(4096)
                    if not packet:
                        break
                    data += packet
                    # print(data)
                images_dict = pickle.loads(data)
                #print(images_dict)
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

def save_images(rgb_image, depth_image):
    cv2.imwrite('rgb_image.png', rgb_image)
    cv2.imwrite('depth_image.png', depth_image)

def load_images():
    rgb_image = cv2.imread('rgb_image.png')
    depth_image = cv2.imread('depth_image.png')
    return rgb_image, depth_image

def start_client(host='10.12.125.172', port=5000):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        while True:
            command = s.recv(1024)
            if command == b'capture':
                rgb_image, depth_image = capture_images()
                save_images(rgb_image, depth_image)
            elif command == b'send':
                rgb_image, depth_image = load_images()
                images_dict = {}
                _, rgb_bytes = cv2.imencode('.png', rgb_image)
                _, depth_bytes = cv2.imencode('.png', depth_image)
                images_dict['rgb'] = rgb_bytes.tobytes()
                images_dict['depth'] = depth_bytes.tobytes()
                data = pickle.dumps(images_dict)
                s.sendall(data)

if __name__ == "__main__":
    server_thread = threading.Thread(target=start_server)
    server_thread.daemon = True
    server_thread.start()
    time.sleep(1)  # Give the server a moment to start

    start_client()
    server_thread.join()