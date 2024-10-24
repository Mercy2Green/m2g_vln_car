roslaunch livox_ros_driver rviz_MID360.launch 

rosbag record /livox/lidar -O /home/uav/m2g_vln_car/vln_car/src/vln/data/4

roslaunch livox_camera_calib bag_to_pcd.launch 

roslaunch livox_camera_calib calib.launch

roslaunch livox_camera_calib multi_calib.launch

rosrun camera_calibration cameracalibrator.py --size 8x11 --square 0.015 image:=/orbbec_camera/color/image_raw

cd /home/uav/m2g_vln_car/SensorsCalibration/lidar2camera/manual_calib/

./bin/run_lidar2camera data/0.png data/0.pcd data/center_camera-intrinsic.json data/top_center_lidar-to-center_camera-extrinsic.json

