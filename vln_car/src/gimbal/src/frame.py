# This file is used to define the frame class, which is used to represent the frame of the gimbal.


class Frame():
    def __init__(self, address, command, data1, data2):
        self.address = address
        self.command = command
        self.data1 = data1
        self.data2 = data2

    def to_bytes(self):
        return bytes([0xFF, self.address, 0x00, self.command, self.data1, self.data2, self.calculate_checksum()])

    def calculate_checksum(self):
        return (self.address + 0x00 + self.command + self.data1 + self.data2) & 0xFF

    @staticmethod
    def from_bytes(frame):
        return Frame(frame[1], frame[3], frame[4], frame[5])
    

def frame_check_horizontal_angle(address):
    return Frame(address, 0x51, 0x00, 0x00).to_bytes()

def frame_control_horizontal_angle(address):
    return Frame(address, 0x4B, 0x00, 0x00).to_bytes()