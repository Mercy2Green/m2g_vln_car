import serial
import threading

from communication import *

try:

    portx = "/dev/ttyUSB0"

    bps = 2400

    timex = 5

    ser = serial.Serial(portx, bps, timeout=timex)

    # 创建并启动读取线程
    read_thread = threading.Thread(target=read_from_port, args=(ser,))
    read_thread.daemon = True
    read_thread.start()

    # 保持主线程运行以便发送数据
    while True:
        user_input = input("Enter the angle. Enter 'exit' to quit. Enter 'check' to check the angle.\n")
        if user_input.lower() == 'exit':
            break
        elif user_input.lower() == 'check':
            data_frame = frame_check_horizontal_angle()
            ser.write(data_frame)
            continue
        else:
            try:
                angle = float(user_input)
                data_frame = frame_control_horizontal_angle(angle)
                ser.write(data_frame)
            except ValueError:
                print("Invalid input. Please enter a number or 'exit'.")

    ser.close()  # 关闭串口

except Exception as e:
    print("---异常---：", e)