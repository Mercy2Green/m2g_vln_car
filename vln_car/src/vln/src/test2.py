#! /usr/bin/env python
import socket
import threading
import cv2
import numpy as np
import time

# Server function
def start_server(host='10.12.125.172', port=5000):
    def handle_client(conn, addr):
        print(f"Connected by {addr}")
        data = b''
        while True:
            packet = conn.recv(4096)
            if not packet:
                print("No more data")
                break
            data += packet
        # print(data)  # Print the received data for debugging
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('Received Image', image)
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
def send_image(host='10.12.125.172', port=5000, image_path='images/rgb_0.png'):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        image = cv2.imread(image_path)
        if image is None:
            print("Failed to load image")
            return
        _, image_bytes = cv2.imencode('.png', image)
        s.sendall(image_bytes.tobytes())

if __name__ == "__main__":
    # Start the server in a separate thread
    server_thread = threading.Thread(target=start_server)
    server_thread.start()  # Do not set as daemon

    # Give the server a moment to start
    time.sleep(1)

    # Send the image from the client
    send_image(image_path='images/rgb_0.png')

    # Wait for the server thread to finish
    server_thread.join()