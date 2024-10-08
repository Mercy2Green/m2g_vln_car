from gimbal_interface import Gimbal_interface
from ranger_mini_2_interface import Ranger_mini_2_interface

from odom_interface import Odom_interface
from image_interface import Image_interface

from sensor_msgs.msg import Image

class VCBot(object):

    def __init__(
        self,
        robot_name="vcbot",
        gimbal_model = None,
        gimbal_h_control_topic = '/gimbal/horiz_control',
        gimbal_v_control_topic = '/gimbal/vert_control',
        gimbal_h_angle_topic = '/gimbal/horiz_angle',
        gimbal_v_angle_topic = '/gimbal/vert_angle',
        base_model = None,
        use_move_base_action=False,
        odom_model = None,
        odom_offset = [0, 0, 0, 0, 0, 0], # x, y, z, yaw, pitch, roll
        odom_sub_topic = "/odom",
        odom_pub_topic = "/odom_transformed", 
        RGB_model = None,
        RGB_WH = [640, 480], # width, height
        RGB_FOV = [60, 60], # HFOV, VFOV
        RGB_sub_topic = "/camera/color/image_raw",
        Depth_model = None,
        Depth_WH = [640, 480],
        Depth_FOV = [60, 60],
        Depth_sub_topic = "/camera/depth/image_raw",
        Depth_upscale = 1.0,
        Depth_downscale = 1.0,
        ):

        self.robot_name = robot_name
        self.gimbal_model = gimbal_model
        self.gimbal_h_control_topic = gimbal_h_control_topic
        self.gimbal_v_control_topic = gimbal_v_control_topic
        self.gimbal_h_angle_topic = gimbal_h_angle_topic
        self.gimbal_v_angle_topic = gimbal_v_angle_topic
        self.base_model = base_model
        self.use_move_base_action = use_move_base_action
        self.odom_model = odom_model
        self.odom_offset = odom_offset
        self.odom_sub_topic = odom_sub_topic
        self.odom_pub_topic = odom_pub_topic
        self.RGB_model = RGB_model
        self.RGB_WH = RGB_WH
        self.RGB_FOV = RGB_FOV
        self.RGB_sub_topic = RGB_sub_topic
        self.Depth_model = Depth_model
        self.Depth_WH = Depth_WH
        self.Depth_FOV = Depth_FOV
        self.Depth_sub_topic = Depth_sub_topic
        self.Depth_upscale = Depth_upscale
        self.Depth_downscale = Depth_downscale

        if odom_model is not None:
            self.odom = Odom_interface(
                name=robot_name,
                sub_topic=odom_sub_topic,
                pub_topic=odom_pub_topic,
                x=odom_offset[0],
                y=odom_offset[1],
                z=odom_offset[2],
                yaw=odom_offset[3],
                pitch=odom_offset[4],
                roll=odom_offset[5],  
            )

        if RGB_model is not None:
            self.rgb = Image_interface(
                image_model=RGB_model,
                image_WH=RGB_WH,
                image_FOV=RGB_FOV,
                image_sub_topic=RGB_sub_topic,
            )
        
        if Depth_model is not None:
            self.depth = Image_interface(
                image_model=Depth_model,
                image_WH=Depth_WH,
                image_FOV=Depth_FOV,
                image_sub_topic=Depth_sub_topic,
                pixel_upscale=Depth_upscale,
                pixel_downscale=Depth_downscale,
            )

        if gimbal_model is not None:
            if gimbal_model == "c40":
                self.gimbal = Gimbal_interface(
                    gimbal_name=robot_name,
                    gimbal_horiz_control_topic=gimbal_h_control_topic,
                    gimbal_vert_control_topic=gimbal_v_control_topic,
                )
            else:
                raise NotImplementedError(
                    "Gimbal model not implemented: " + gimbal_model
                )

        if base_model is not None:
            if base_model == "ranger_mini_2":
                self.base = Ranger_mini_2_interface(
                    robot_name=robot_name,
                    use_move_base_action=use_move_base_action,
                )
            else:
                raise NotImplementedError(
                    "Move model not implemented: " + base_model
                )
            
        