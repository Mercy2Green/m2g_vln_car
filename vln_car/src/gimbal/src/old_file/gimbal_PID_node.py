#!/usr/bin/env python

from time import sleep
import rospy
import threading
from std_msgs.msg import Float32, Float32MultiArray
import serial

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative

        # Calculate output
        output = P + I + D

        # Update previous error
        self.previous_error = error

        return output

class GimbalPIDNode:
    def __init__(self, portx="/dev/ttyUSB1", bps=2400, timex=5, address=1):
        rospy.init_node('GimbalPIDNode', anonymous=True)
        
        # Serial port setup
        self.portx = portx
        self.bps = bps
        self.timex = timex
        self.ser = serial.Serial(self.portx, self.bps, timeout=self.timex)
        self.address = address

        buffer_length = 3
        frame_length = 7

        self.buffer = bytearray(buffer_length * frame_length)  # Buffer to accumulate received data
        self.frame = None
        self.frame_lock = threading.Lock()

        self.target_angle = None
        
        # Publisher
        self.horizontal_angle_pub = rospy.Publisher('gimbal_horizontal_angle', Float32, queue_size=10)
        self.vertical_angle_pub = rospy.Publisher('gimbal_vertical_angle', Float32, queue_size=10)
        self.target_angle_pub = rospy.Publisher('gimbal_target_angle', Float32, queue_size=1)

        self.turn_sub = rospy.Subscriber('control_angle', Float32MultiArray, self.receive_angle_and_turn)
        self.target_angle_sub = rospy.Subscriber('gimbal_target_angle', Float32, self.current_target_angle)
        # The angle is a list of two elements, the first is the horizontal angle, the second is the vertical angle
        
        # PID Controllers
        self.horizontal_pid = PIDController(Kp=0.05, Ki=0.01, Kd=0.05, setpoint=0.0)

        # Start threads
        self.start_gimbal()

    def start_gimbal(self):
        rospy.loginfo("Starting gimbal PID node")

        read_port_thread = threading.Thread(target=self.read_from_port, args=(self.ser,))
        read_port_thread.daemon = True
        read_port_thread.start()

        angle_pub_thread = threading.Thread(target=self.get_angle_and_publish, args=(self.horizontal_angle_pub, self.vertical_angle_pub))
        angle_pub_thread.daemon = True
        angle_pub_thread.start()

        while not rospy.is_shutdown():

            target_angle = self.test_control_angle(axis='horizontal')
            self.target_angle_pub.publish(target_angle)
            sleep(0.5) # This is essential cause the device need time to fresh the self.target_angle
            if target_angle is not None:
                self.target_angle_pub.publish(target_angle)
                self.horizontal_pid.setpoint = target_angle
                cur_angle = None
                while not self.get_angle_and_check(target_angle, 0.01, True):
                    if self.frame:
                        frame = self.frame
                        cur_angle = get_angle_from_frame(frame)
                        if cur_angle is not None:
                            print('Current angle:', cur_angle)
                            if abs(cur_angle - target_angle) < 1:
                                pid_angle = self.horizontal_pid.update(cur_angle, dt=0.1) + cur_angle # Why add cur_angle ?
                                print('PID angle:', pid_angle)
                                control_data_frame = frame_control_angle(angle=pid_angle, axis='horizontal')
                                self.ser.write(control_data_frame)
                    check_angle_frame = frame_check_angle(axis='horizontal', address=self.address)
                    self.ser.write(check_angle_frame)
                    sleep(0.1)
            else:
                continue

        # while not rospy.is_shutdown():

        #     target_angle = self.test_control_angle(axis='horizontal')
        #     self.target_angle_pub.publish(target_angle)
        #     sleep(0.5) # This is essential cause the device need time to fresh the self.target_angle

        #     if self.target_angle is not None:
        #         self.target_angle_pub.publish(target_angle)
        #         self.horizontal_pid.setpoint = target_angle
        #         current_angle = None
        #         while not self.get_angle_and_check(target_angle, 0.0001, True):
        #             check_angle_frame = frame_check_angle(axis='horizontal', address=self.address)
        #             self.ser.write(check_angle_frame)
        #             if self.frame:
        #                 frame = self.frame
        #                 current_angle = get_angle_from_frame(frame)
        #             if current_angle is not None:
        #                 control_signal = self.horizontal_pid.update(current_angle, dt=0.1)
        #                 print('Control signal:', control_signal)
        #                 control_data_frame = frame_control_angle(angle=control_signal, axis='horizontal')
        #                 self.ser.write(control_data_frame)
        #                 sleep(0.1)
        #     else:
        #         sleep(0.5)

    def current_target_angle(self, msg):
        if msg.data is not None:
            self.target_angle = msg.data

    def receive_angle_and_turn(self, msg): 
        # The first angle is horizontal angle, the second angle is vertical angle
        h_angle, v_angle = msg.data
        h_data_frame = frame_control_angle(h_angle, axis='horizontal', address=self.address)
        self.ser.write(h_data_frame)
        v_data_frame = frame_control_angle(v_angle, axis='vertical', address=self.address)
        self.ser.write(v_data_frame)

    def test_control_angle(self, axis='horizontal'):
        """
        This function waits for user input angle from the keyboard and sends the control command to the gimbal.
        """
        try:
            angle = float(input(f"Enter the {axis} angle to control the gimbal: "))
            data_frame = frame_control_angle(angle=angle, axis=axis)
            self.ser.write(data_frame)
            rospy.loginfo(f"Sent control command for {axis} angle: {angle}")
            return angle
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid number.")
            return None
    
    def get_current_angle(self):
        with self.frame_lock:
            if self.frame:
                return get_angle_from_frame(self.frame)
        return None

    def get_angle_and_check(self, target_angle=None, acc=0.05, sub=False):
        if sub == False:
            angle_range = [target_angle - acc, target_angle + acc]
        else:
            target_angle = self.target_angle
            if target_angle is not None:
                angle_range = [target_angle - acc, target_angle + acc]
            else:
                return False
        if self.frame:
            frame = self.frame
            cur_angle = get_angle_from_frame(frame)
            if cur_angle is not None and cur_angle >= angle_range[0] and cur_angle <= angle_range[1]:
                return True
            else:
                return False
                
    def get_angle_and_publish(self, h_pub, v_pub, horizontal_flag=True, vertical_flag=True):
        while not rospy.is_shutdown():
            if self.frame:
                frame = self.frame
                if frame[3] == 0x59 and horizontal_flag:
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        h_pub.publish(angle)
                    else:
                        rospy.logwarn("Failed to get horizontal angle from frame")
                if frame[3] == 0x5B and vertical_flag:
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        v_pub.publish(angle)
                    else:
                        rospy.logwarn("Failed to get vertical angle from frame")
    
    def read_from_port(self, ser):
        while not rospy.is_shutdown():
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                self.buffer.extend(data)  # Add received data to the buffer
                while len(self.buffer) >= 7:  # Assuming a frame is 7 bytes long
                    if self.buffer[0] == 0xFF:  # Check for the start of the frame
                        frame = self.buffer[:7]  # Extract the frame
                        self.buffer = self.buffer[7:]  # Remove the extracted frame from the buffer
                        self.frame = frame
                    else:
                        self.buffer.pop(0)  # Remove the first byte if it's not the start of a frame

    def run(self):
        rospy.spin()
        self.ser.close()

#### Communication Part

def frame_control_horizontal_angle(angle, address=1):
    angle_int = int(angle * 100) 
    data1 = (angle_int >> 8) & 0xFF
    data2 = angle_int & 0xFF
    hex_address = int(address)
    checksum = calculate_checksum(hex_address, 0x00, 0x4B, data1, data2)
    return bytes([0xFF, hex_address, 0x00, 0x4B, data1, data2, checksum])

def frame_control_vertical_angle(angle, address=1):
    angle_int = int(angle * 100) 
    data1 = (angle_int >> 8) & 0xFF
    data2 = angle_int & 0xFF
    hex_address = int(address)
    checksum = calculate_checksum(hex_address, 0x00, 0x4D, data1, data2)
    return bytes([0xFF, hex_address, 0x00, 0x4D, data1, data2, checksum])

def frame_check_horizontal_angle(address=1):
    hex_address = int(address)
    return bytes([0xFF, hex_address, 0x00, 0x51, 0x00, 0x00, 0x52])

def frame_check_vertical_angle(address=1):
    hex_address = int(address)
    return bytes([0xFF, hex_address, 0x00, 0x53, 0x00, 0x00, 0x54])

def frame_control_angle(angle, axis='horizontal', address=1):
    hex_address = int(address)
    if axis == 'horizontal':
        return frame_control_horizontal_angle(angle, hex_address)
    elif axis == 'vertical':
        return frame_control_vertical_angle(angle, hex_address)
    else:
        rospy.logwarn("Invalid angle. Please enter 'horizontal' or 'vertical'.")

def frame_check_angle(axis='horizontal', address=1):
    if axis == 'horizontal':
        return frame_check_horizontal_angle(address)
    elif axis == 'vertical':
        return frame_check_vertical_angle(address)
    else:
        rospy.logwarn("Invalid angle. Please enter 'horizontal' or 'vertical'.")

def calculate_checksum(*args):
    sum = 0
    for value in args:
        sum += value
    return sum & 0xFF

#### Process Part

def get_angle_from_frame(bytes_frame):
    axis_flag = bytes_frame[3]
    if axis_flag != 0x59 and axis_flag != 0x5B:
        rospy.logwarn("Invalid frame. Please check the frame.")
        return None
    
    data1 = bytes_frame[4]
    data2 = bytes_frame[5]
    angle_int = (data1 << 8) + data2

    checksum = calculate_checksum(bytes_frame[1], bytes_frame[2], bytes_frame[3], bytes_frame[4], bytes_frame[5])
    if checksum != bytes_frame[6]:
        rospy.logwarn("Checksum mismatch")
        return None

    return angle_int / 100

if __name__ == '__main__':
    try:
        node = GimbalPIDNode(portx="/dev/ttyUSB1", bps=2400, timex=5, address=1)
        node.run()
    except rospy.ROSInterruptException:
        pass