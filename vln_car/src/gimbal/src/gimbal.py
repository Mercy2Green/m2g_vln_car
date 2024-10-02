# This file is used to define the gimbal class.

# This is independent of the ROS.



from frame import *
import serial

class Gimbal():
    def __init__(
            self,            
            portx="/dev/ttyUSB1",
            bps=2400,
            timex=5,
            address=1, 
            speed=0x1E):

        self.address = address
        self.serial = serial.Serial(portx, bps, timeout=timex)
        self.speed = speed

    def set_speed(self, speed):
        # The speed range is 0x01 to 0x3F. To the decimal system, the range is 1 to 63.
        if speed > 0x3F or speed < 0x01:
            raise ValueError("The speed range is 0x01 to 0x3F.")
        else:
            self.speed = speed

    def get_angle_from_frame(self, frame: Frame):
        bytes_frame = frame.to_bytes()
        axis_flag = bytes_frame[3]
        if axis_flag != 0x59 and axis_flag != 0x5B:
            print("Invalid frame. Please check the frame.")
            return None
        # the frame is a bytes object
        data1 = bytes_frame[4]
        data2 = bytes_frame[5]
        if axis_flag == 0x59:
            angle = ((data1 << 8) + data2) / 100
        elif axis_flag == 0x5B:
            angle = ((data1 << 8) + data2) / 100
            if angle > 0:
                angle = 360 - angle
        # also need to check the checksum
        checksum = frame.calculate_checksum()
        if checksum != bytes_frame[6]:
            Warning("Checksum mismatch")
            return None
        return angle

    def check_horizontal_angle(self):
        frame = Frame_check_horizontal_angle(self.address)
        self.serial.write(frame.to_bytes())
        angle = self.get_angle_from_frame(Frame.from_bytes(self.serial.read(7)))
        return angle

    def check_vertical_angle(self):
        frame = Frame_check_vertical_angle(self.address)
        self.serial.write(frame.to_bytes())
        angle = self.get_angle_from_frame(Frame.from_bytes(self.serial.read(7)))
        return angle
    
    def read_serial(self):
        return Frame.from_bytes(self.serial.read(7))

    def control_horizontal_angle(self, angle):
        frame = Frame_control_horizontal_angle(self.address, angle)
        self.serial.write(frame.to_bytes())
        print(frame.to_bytes().hex().upper())
        return True

    def control_vertical_angle(self, angle):
        frame = Frame_control_vertical_angle(self.address, angle)
        self.serial.write(frame.to_bytes())
        return True

    def control_stop(self):
        frame = Frame_control_stop(self.address)
        self.serial.write(frame.to_bytes())
        return True

    def control_up(self, speed=0x1E):
        frame = Frame_control_up(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_down(self, speed=0x1E):
        frame = Frame_control_down(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_left(self, speed=0x1E):
        frame = Frame_control_left(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_right(self, speed=0x1E):
        frame = Frame_control_right(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_left_up(self, speed=0x1E):
        frame = Frame_control_left_up(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_right_up(self, speed=0x1E):
        frame = Frame_control_right_up(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True

    def control_left_down(self, speed=0x1E):
        frame = Frame_control_left_down(self.address, speed)
        self.serial.write(frame.to_bytes())
        return True