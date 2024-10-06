import rospy

##



#### frame part

# Data frame is FF add 00 4B data1 data2 checksum
# add is the address of the deivce, the default value is 01
# data1 is the high 8 byte of the angle
# data2 is the low 8 byte of the angle
# checksum is the low 8 byte of the sum of 2 to 6 byte

def frame_control_horizontal_angle(angle, address=1):
    # the angle is the degree of the gimbal, it is a decimal number
    angle_int = int(angle*100) 
    data1 = (angle_int >> 8) & 0xFF
    data2 = angle_int & 0xFF
    hex_address = int(address)
    checksum = calculate_checksum(hex_address, 0x00, 0x4B, data1, data2)
    return bytes([0xFF, hex_address, 0x00, 0x4B, data1, data2, checksum])

def frame_check_horizontal_angle(address=1):
    hex_address = int(address)
    return bytes([0xFF, hex_address, 0x00, 0x51, 0x00, 0x00, 0x52])

def calculate_checksum(*args):
    # the frame is a bytes object
    sum = 0
    for value in args:
        sum += value
    return sum & 0xFF

def read_from_port(ser):
    buffer = bytearray()  # Buffer to accumulate received data
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)  # Add received data to the buffer
            while len(buffer) >= 7:  # Assuming a frame is 7 bytes long
                if buffer[0] == 0xFF:  # Check for the start of the frame
                    frame = buffer[:7]  # Extract the frame
                    buffer = buffer[7:]  # Remove the extracted frame from the buffer
                    # convert the frame to angle
                    frame = frame.hex().upper()
                    angle = get_angle_from_frame(bytes.fromhex(frame))
                    print(f"Received angle: {angle}")
                else:
                    buffer.pop(0)  # Remove the first byte if it's not the start of a frame


def read_from_port_ros(ser, pub):
    buffer = bytearray()  # Buffer to accumulate received data
    while not rospy.is_shutdown():
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)  # Add received data to the buffer
            while len(buffer) >= 7:  # Assuming a frame is 7 bytes long
                if buffer[0] == 0xFF:  # Check for the start of the frame
                    frame = buffer[:7]  # Extract the frame
                    buffer = buffer[7:]  # Remove the extracted frame from the buffer
                    # Convert the frame to angle
                    angle = get_angle_from_frame(frame)
                    if angle is not None:
                        print(f"Received angle: {angle}")
                        pub.publish(angle)
                else:
                    buffer.pop(0)  # Remove the first byte if it's not the start of a frame


#### process part

def get_angle_from_frame(bytes_frame):
    # the frame is a bytes object
    data1 = bytes_frame[4]
    data2 = bytes_frame[5]
    angle_int = (data1 << 8) + data2

    # also need to check the checksum
    checksum = calculate_checksum(bytes_frame[1], bytes_frame[2], bytes_frame[3], bytes_frame[4], bytes_frame[5])
    if checksum != bytes_frame[6]:
        Warning("Checksum mismatch")
        return None

    return angle_int / 100

#### device class

class GIMBAL():

    def __init__(
        self,
        address):
        self.address = address

    def send_frame(
        self,
        frame):
        pass

    def read_frame(
        self,
        frame):
        pass

#### Test part

def test_angle(angle=0):
    data_frame = frame_control_horizontal_angle(angle)

    # convert data_frame to str like "FF 01 00 4B 00 00 4C"
    data_frame = " ".join([f"{byte:02X}" for byte in data_frame])

    print(data_frame)



if __name__ == "__main__":
    test_angle(180)