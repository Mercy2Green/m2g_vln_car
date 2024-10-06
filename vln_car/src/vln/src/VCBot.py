from gimbal_interface import Gimbal_interface
from ranger_mini_2_interface import Ranger_mini_2_interface

class VCBot(object):

    def __init__(
        self,
        camera_model = None,
        laser_model = None,
        gimbal_model = None,
        base_model = None,
        camera_group_name="camera",
        laser_group_name="laser",
        gimbal_group_name="gimbal",
        move_group_name="move",
        use_move_base_action=False,
        robot_name="vcbot",):

        if camera_model is not None:
            if camera_model == "realsense":
                from realsense_interface import Realsense_interface
                self.camera = Realsense_interface(
                    camera_name=robot_name,
                )
            elif camera_model == "orbbec":
                from orbbec_interface import Orbbec_interface
                self.camera = Orbbec_interface(
                    camera_name=robot_name,
                )
            else:
                raise NotImplementedError(
                    "Camera model not implemented: " + camera_model
                )
        
        if laser_model is not None:
            if laser_model == "livoxs":
                from livox_interface import Livox_interface
                self.laser = Livox_interface(
                    laser_name=robot_name,
                )
        
        if gimbal_model is not None:
            if gimbal_model == "c40":
                self.gimbal = Gimbal_interface(
                    gimbal_name=robot_name,
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