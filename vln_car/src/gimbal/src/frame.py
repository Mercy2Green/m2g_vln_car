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
        try:
            return Frame(frame[1], frame[3], frame[4], frame[5])
        except:
            return None
    

class Frame_check_horizontal_angle(Frame):
    def __init__(self, address):
        super().__init__(address, 0x51, 0x00, 0x00)

class Frame_check_vertical_angle(Frame):
    def __init__(self, address):
        super().__init__(address, 0x52, 0x00, 0x00)

class Frame_control_horizontal_angle(Frame):
    def __init__(self, address, angle):
        angle = int(angle * 100)
        data1 = angle >> 8 & 0xFF
        data2 = angle & 0xFF
        super().__init__(address, 0x4B, data1, data2)

class Frame_control_vertical_angle(Frame):
    def __init__(self, address, angle):
        if angle < 0:
            angle = int(angle * 100)
            data1 = angle >> 8 & 0xFF
            data2 = angle & 0xFF
        else:
            angle = 36000 - int(angle * 100)
            data1 = angle >> 8 & 0xFF
            data2 = angle & 0xFF
        super().__init__(address, 0x4D, data1, data2)

class Frame_control_stop(Frame):
    def __init__(self, address):
        super().__init__(address, 0x00, 0x00, 0x00)

class Frame_control_up(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x08, 0x00, speed)

class Frame_control_down(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x10, 0x00, speed)

class Frame_control_left(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x04, speed, 0x00)

class Frame_control_right(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x02, speed, 0x00)

class Frame_control_left_up(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x0A, speed, speed)

class Frame_control_left_down(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x14, 0x3F, speed)

class Frame_control_right_up(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x0A, speed, speed)

class Frame_control_right_down(Frame):
    def __init__(self, address, speed=0x1E):
        super().__init__(address, 0x12, 0x3F, speed)

# 水平角度:
# 控制指令 :FF add 00 4B dat1 dat2 SUM
# add=云台地址
# dat1=预设角度高8位，
# dat2=预设角度底8位，
# SUM=代表校验和,即第2至6位和的低8位,
# dat1<<8 + dat2 = 角度*100
# 例如:设置地址1的云台到180度的位置的指令码为:FF 01 00 4B 46 50 E2  
# 水平回传的指令为:FF add 00 59  dat1 dat2 SUM
# 水平角度查询指令:FF add 00 51 00 00 SUM

# 垂直角度:
# 控制指令:FF add 00 4D dat1 dat2 SUM
# add=云台地址
# dat1=预设角度高8位，
# dat2=预设角度底8位，  
# 当角度为负时，dat1<<8 + dat2 = 角度*100
# 当角度为正时，dat1<<8 + dat2 = 36000 - 角度*100
# 例如:设置地址1的云台到-20度的位置的指令码为:FF 01 00 4D 07 D0 25  
# 设置地址1的云台到20度的位置的指令码为:FF 01 00 4D 84 D0 A2 
# 垂直回传指令为:FF add 00 5B  dat1 dat2 SUM
# 垂直角度查询指令:FF add 00 53 00 00 SUM    

# Direction control commands:
# Stop: FF 01 00 00 00 00 01 
# Up: FF 01 00 08 00 1E 27 
# Down: FF 01 00 10 00 1E 2F 
# Left: FF 01 00 04 1E 00 23 
# Right: FF 01 00 02 1E 00 21
# Left Up: FF 01 00 02 1E 00 21
# Left Down: FF 01 00 14 3F 1E 72
# Right Up: FF 01 00 0A 1E 1E 47
# Right Down: FF 01 00 12 3F 1E 70

# Speed level control commands
# (Note: Speed is only effective in basic control. In other modes such as direct angle control, recall preset, or cruise, speed settings are ineffective!)
# Example: FF add 00 04 dat1 00 SUM, where dat1 ranges from 01 to 3F, with 01 being the lowest speed level and 3F being the highest speed level.
# Example: Left FF 01 00 04 01 00 06 (Level 1)


# 坐标位置控制代码说明:
# 以下为坐标位置代码(以下代码以PD协议为例)
# 00  执行水平:FF 01 00 4B 00 00 4C (连续旋转)
# 0.1 执行水平:FF 01 00 51 00 00 52 
# FF 01 00 4B 00 0A 56 
# 1执行水平:FF 01 00 4B 00 64 B0 
# 2执行水平:FF 01 00 4B 00 C8 14 
# 3执行水平:FF 01 00 4B 01 2C 79 
# 4执行水平:FF 01 00 4B 01 90 DD 

# 坐标位置案例解释
# 0.33 :  FF 01 00 5B 8C 7F 67 36000-35967
# 坐标代码要从十六进制转为转换10进制,然后再转换为16进制,安装协议填入
# 代码示例:第一个取 8c 7f，转换10进制为35967，角度＝36000-35967=33，再除以100为0.33,然后需要反过来转换，安装协议填入

# 0: FF 01 00 5B 00 00 5C 

# 20.6: FF 01 00 5B 84 CA AA  36000-33994

# -20: FF 01 00 5B 07 D1 34 2001

# 1.50: FF 01 00 59 00 96 F0  150

# 90: FF 01 00 59 23 28 A5  9000

# 359:FF 01 00 59 8C 3C 22 35900



# 巡航控制指令:
# 预置位及自动巡航
# 预置位:(设置预置位后,坐标位置会自动添加到巡航线路中,巡航中途断电-需重新调用启动指令)
# 设置预制位：FF add 00 03  00 dat1 SUM；dat1取值由01--FF，如设置001号预置位：FF 01 00 03 00 01 05 
# 召回预制位：FF add 00 07  00 dat1 SUM；dat1取值由01--FF，如召回255号预置位：FF 01 00 07 00 FF 07
# 启动自动巡航：FF 01 00 07 00 62 6A (巡航中途断电-需重新调用启动指令)

# 水平巡航控制指令:
# 水平自动巡航:(扫描需要先设置起点，再设置终点，之后启动水平自动巡航)
# 设置水平巡航起点: FF 01 00 03 00 5C 60 
# 设置水平巡航终点:: FF 01 00 03 00 5D 61
# 启动水平自动巡航: FF 01 00 07 00 63 6B  (巡航中途断电-需重新调用启动指令)

# 部份控制代码示例(PELCO-D协议，01号地址)
# 该代码,由配套的上位机软件自动生成,可以直接复制,以下代码仅仅是部分示例,PELCO-D协议，01号地址)
# PELCO-D协议，01号地址的设置和调用预置位的代码如下
# 设置1号预置位： FF 01 00 03 00 01 05   
# 设置2号预置位： FF 01 00 03 00 02 06   
# 设置3号预置位： FF 01 00 03 00 03 07   
# 设置4号预置位： FF 01 00 03 00 04 08   
# 设置5号预置位： FF 01 00 03 00 05 09   
# 设置6号预置位： FF 01 00 03 00 06 0A   
# 调用1号预置位： FF 01 00 07 00 01 09   
# 调用2号预置位： FF 01 00 07 00 02 0A   
# 调用3号预置位： FF 01 00 07 00 03 0B   
# 调用4号预置位： FF 01 00 07 00 04 0C   
# 调用5号预置位： FF 01 00 07 00 05 0D   
# 调用6号预置位： FF 01 00 07 00 06 0E  
# 打开1号开关:FF 01 00 09 00 01 0B
# 关闭1号开关:FF 01 00 0B 00 01 0D
# 打开2号开关:FF 01 00 09 00 01 0B
# 关闭2号开关:FF 01 00 0B 00 01 0D  
# 召回1号位置: FF 01 00 07 00 01 09
# 启动自动扫描: FF 01 00 07 00 63 6B 
# 启动多点巡航: FF 01 00 07 00 62 6A
# 查询水平角度: FF 01 00 51 00 00 52
# 查询垂直角度：FF 01 00 53 00 00 54
# 巡航起点: FF 01 00 03 00 5C 60(设92)
# 巡航终点: FF 01 00 03 00 5D 61(设93)
# 启动水平巡航: FF 01 00 07 00 63 6B(调99)
# 启动多点巡航: FF 01 00 07 00 62 6A(调98)
# 停止所有巡航: FF 01 00 00 00 00 01

# 方向控制指令:
# 停止:FF 01 00 00 00 00 01 
# 向上:FF 01 00 08 00 1E 27 
# 向下:FF 01 00 10 00 1E 2F 
# 向左:FF 01 00 04 1E 00 23 
# 向右:FF 01 00 02 1E 00 21
# 左上:FF 01 00 02 1E 00 21
# 左下:FF 01 00 14 3F 1E 72
# 左上:FF 01 00 0A 1E 1E 47
# 右下:FF 01 00 12 3F 1E 70